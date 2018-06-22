//DIY 공기청정기: Clean UP! Program 소스
//MCU: ATMEAG328P / 16MHz / 5V 사용
// 컴파일러: AVRSTUDIO7
// ===================================================
//Ver v1.1  , 제작: SED(SeMin DIY)
//20180606 UPDATE
// ===================================================
//Ver v2.0  , 제작: SED(SeMin DIY)
// 추가된기능
// ===================================================
//Ver v3.0  , 제작: SED(SeMin DIY)
// 추가된기능
// 1. 안정화된 미세먼지 센싱
// 2. 수동 캘리브레이션 모드 동작 오류 수정
// 3. 블루투스 데이터 전송 방식 수정
//
// ===================================================
// v3.1, 수정: Bryan (byoungsu.kr@gmail.com) 2018.06.18
// 1. 공기 상태 등급 3단계 4단계 업데이트
// 2. 공기 센서링 샘플링 개수 30 -> 10 수정
// 3. FAN 스피드 3단계 4단계 업데이트
// 4. LED 색상 업데이트( 4단계 조정 )
// 5. 캘리브레이션 과정중 예외 업데이트(보정)
// 6. 통신인터페이스 위한 디버깅 정보 주석
//
// ===================================================
// v3.2, 수정: Bryan (byoungsu.kr@gmail.com) 2018.06.21
// 1. 캘리브레이션 오프셋 적용 ( 0~20 )
// 2. 동작시간 카운트 변수 2개 추가
// 3. 동작 시간에 대한 리셋 명령 추가
// 4. 캘리브레이션 LED 색상 변경
//
//
//상업적 금지! , 원작자 표시 조건하에 공유 및 수정 가능!

#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
 
#include "ADC.h"
#include "uart_328p.h"
#include "ws2812b.h"
#define ON 1
#define OFF 0
 
#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))
//--------------------------------
#define LED_NUM 			4
#define SENSOR_LED_ON 		clr_bit(PORTD,7)
#define SENSOR_LED_OFF 		set_bit(PORTD,7)


 
#define R 0
#define G 1
#define B 2
#define WS2812B_LED_NUM 4

#define LED_GRADE_MIN		0
#define LED_GRADE_5			5
#define LED_GRADE_MAX		10

#define DUST_GRADE_GOOD 	15
#define DUST_GRADE_NORMAL 	35
#define DUST_GRADE_BAD		75

#define EEPROM_LED_LIGHT 		0x40
#define EEPROM_COM_MODE 		0x30
#define EEPROM_ALIVE_DAY		0x28
#define EEPROM_ALIVE_TOK		0x20
#define EEPROM_CAL_OFFSET		0x08
#define EEPROM_CAL_DATA 		0x00

#define DEFAULT_DUST_CALIBRATION	0.105000
 
#define FAN_SPEED_MAX 		0xFFFF
#define FAN_SPEED_BAD		750
#define FAN_SPEED_NORMAL 	500
#define FAN_SPEED_GOOD 		250
//---------------------------------
#define COM_AUTO 			0x0
#define COM_FAN_1 			0x1
#define COM_FAN_2 			0x2
#define COM_FAN_3 			0x3
#define COM_CALIBRATION 	0x4
#define COM_RESET			0x5
//---------------------------------
#define RX_WRITE_ENABLE 			0xFD
#define RX_WRITE_DISABLE 			0xFB
#define RX_COM 				1
#define RX_LED_LIGHT 		2
#define RX_CLEAN_ENABLE 	3
#define RX_CLEAN_DATA 		4
#define RX_CLEAN_OFFSET		5
//---------------------------------
#define AVG_DUST_CNT 				10

char command0[50];	// 30 -> 50
char rx_db[10];
unsigned char rx_counter=0;
char rx_write_enable=0;
 

unsigned char button_click=0; //버튼 클릭 시간 체크


//--------------------
unsigned int fan_speed_db[4]={FAN_SPEED_GOOD,FAN_SPEED_NORMAL,FAN_SPEED_BAD,FAN_SPEED_MAX};
unsigned char led_blink_counter=0; //led 몇번 깜박일지
 
unsigned char timer_counter=0;

// 모드 색기준 자동:분홍 / 속도1: 녹색 / 속도2: 파랑 / 속도3: 빨강 / 셋업: 주황색 
unsigned char setup_color[5][3]={ {255,0,221}, {0,255,0}, {0,0,255}, {255,0,0}, {255,187,0} };
unsigned char sensor_color[4][3]={ {0,0,255}, {0,255,0}, {255,255,0},{255,0,0}}; //맨 마지막은 노란색 , 캘리용
unsigned char led_color_buffer[3];
unsigned char led=0;            
char eep_write_enable = 0;

// ---------------------------------
uint8_t com_mode = 0;
uint8_t led_light = 0;
unsigned char dust_grade = 0; 	// 좋음(0), 보통(1), 나쁨(2), 매우나쁨(3).
int pm_density = 0;			// 좋음 0~15ug/m3, 보통 16 ~ 35ug/m3, 나쁨 36 ~ 75ug/m3, 매우나쁨 76이상.
float dustVout = 0.0f;
float cali_data = 0.0f;
uint8_t cali_offset = 0;			// 0 ~ 20
uint8_t alive_day = 0;
uint32_t alive_tok = 0;
//----------------------------------
/*/
시작비트: 0XFD
끝비트: 0XFB
 
1비트: Run_mode(0~5)
2비트: LED 밝기제어 (0 ~ 10)
3비트: 캘리브레이션 ENABLE 명령어
4비트: 캘리브레이션 값(m3ug)
 
*/
 
void calibration_clean_db(char enable, int set_m3ug)
{
    //디지털 센서 값을 보고 캘리브레이션 하는 방식
    //공식 clean_db 값 = 현재값 - (m3ug(블루투스로 전송받은값) * 0.005);
    
    if( enable == ON ) {
		cali_data = dustVout - (set_m3ug * 0.005);
    }
}

ISR(USART_RX_vect)
{
	char RX;
	RX=UDR0;

	if( RX == RX_WRITE_ENABLE ) { 
		rx_write_enable = ON;
	} else if( RX == RX_WRITE_DISABLE ) {
		rx_write_enable = OFF; 
		rx_counter = 0;
		//sprintf(command0,"ok/ %d,%d,%d\r\n",rx_db[0],rx_db[1],rx_db[2]); TX0_STR(command0); //테스트용
		com_mode = rx_db[RX_COM];
		led_light = rx_db[RX_LED_LIGHT];

		if( com_mode == COM_CALIBRATION ) cali_offset = rx_db[RX_CLEAN_OFFSET];
		
		if( com_mode == COM_RESET ) {
			cali_data = DEFAULT_DUST_CALIBRATION;
			cali_offset = 0;
			alive_tok = 0;
			alive_day = 0;
			com_mode = COM_AUTO;
		}

		calibration_clean_db( rx_db[RX_CLEAN_ENABLE], rx_db[RX_CLEAN_DATA] );
		rx_db[RX_CLEAN_ENABLE] = OFF;
		eep_write_enable = ON;
	}

	if( rx_write_enable == ON ) {
		if( rx_counter == RX_COM ) {
			if( !(RX == rx_db[RX_COM] )) { //run mode 가 기존값과 같지 않다면
				led_blink_counter = 5; //led 5회 깜빡여라
			}
		}
		rx_db[rx_counter++] = RX;
		if( rx_counter > 9 ) rx_counter = 0;
	}            
}
 
ISR(TIMER0_COMPA_vect) 
{
	int i = 0;
    if( ++timer_counter == 20 ) { // 5ms x 70 = 350ms    
        if( led_blink_counter != 0  ) {
            if( ++led == 2 ){
                ws2812b_color(0,0,0,4);
                led=0;
            } else {
            	if( com_mode == COM_RESET ) { com_mode = COM_AUTO; }
                for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[com_mode][i] * (led_light * 0.1); } //밝기 조절
                ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
                led_blink_counter--;
            } 
        } else {
            for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_grade][i] * (led_light * 0.1); } //밝기 조절
            ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
        }
    	timer_counter=0;
    }
}

float getAvgDustVoltage() 
{
	unsigned long sumVoutReadValue = 0;
	unsigned long avgVoutReadValue = 0;
	int qVoutReadValue[AVG_DUST_CNT];
	int i = 0;
	float Vo = 0.0f;

	for( i = 0; i < AVG_DUST_CNT; i++ ) {
		SENSOR_LED_ON;
		_delay_us(280);

		while(!(ADCSRA&0x10));
		qVoutReadValue[i] = ADCW + cali_offset;

		_delay_us(40);
		SENSOR_LED_OFF;

		_delay_ms(9);		// 9sec Off.
	}

	for ( i = 0; i < AVG_DUST_CNT; i++) {
		sumVoutReadValue = sumVoutReadValue + qVoutReadValue[i];
	}
	avgVoutReadValue = sumVoutReadValue / AVG_DUST_CNT;
	Vo = (float)((avgVoutReadValue * 5.0) / 1024.0);
	
	if( Vo < cali_data ) {		// Vo 측정값이 Voc 보다 작을 경우 보정 처리.
		cali_data = Vo;
	}
	return Vo;
}

// u3mg 평균화 작업후 단위로 리턴.
int getDustDensity(void) 
{
	int i = 0, sum_num = AVG_DUST_CNT;
	int sum_dust = 0, dust[AVG_DUST_CNT];
	 
	for( i = 0; i < sum_num; i++ ) {
		dustVout = getAvgDustVoltage();
		dust[i] = (dustVout -  cali_data) / 0.005;		// ug
	}

	for( i = 0; i < sum_num; i++) {
		sum_dust = sum_dust + dust[i];
	}
	sum_dust = sum_dust / sum_num;

	return sum_dust;
}
 
 
int main(void){
    // ** Port Number: PD7=Sensor LED
    // PC0(ADC0) = Sensor INPUT 
    // PB1(OC1A)=FAN 
    // PD2(INT0) = BUTTON 
    // PB0 = WS2812B 
    DDRD=0b11111011; DDRB=0XFF;
    uart_init(1);
    ADC_init();
    
    TCCR0A= ((1<<WGM01) | (1<<WGM00)); //CTC 모드 사용
    TCCR0B= ((1<<CS02) | (0<<CS01) | (1<<CS00)); //1024분주
    //001=NO // 010=8 // 011=64 //100=256 // 101=1024
    OCR0A=78;  //5ms
    TIMSK0=((1<<OCIE0A)); 
    
    TCCR1A =0B10000011;
    TCCR1B =0B00001100; //FAST PWM 8분주
    OCR1A=0; //초기에는 fan 끄기//FFFF = 강 / 중 500 / 약 250 
    
    SREG=0x80;		// Global Interrupt Enable

	cali_offset = 0;
	alive_day = 0;
	alive_tok = 0;
    
    //원리: 320ns 마다 led가 on 되어야 하고 280ns 지나는 시점에 ADC 센싱해야된다.
    //up/m3 공식 미세먼지 농도[μg/m³] = (Vo ? Voc) / 0.005;
    //미세먼지 기준 좋음: 0~15ug/m3 / 보통 16 ~ 35ug/m3 / 나쁨 36이상
    
    led_light = eeprom_read_byte((uint8_t*)EEPROM_LED_LIGHT);
    
    if( led_light > LED_GRADE_MAX || led_light == LED_GRADE_MIN ) {
		// run 값이 10보다 크면 즉 ISP 로 처음 넣을때 EEPROM 값은 255이다. (initialize)
        com_mode = COM_AUTO;
        cali_data = DEFAULT_DUST_CALIBRATION;
		cali_offset = 0;
        led_light = LED_GRADE_5;
        
        eeprom_write_float((float*)EEPROM_CAL_DATA, cali_data);
		eeprom_write_byte((uint8_t*)EEPROM_CAL_OFFSET, (uint8_t)cali_offset);
		eeprom_write_dword((uint32_t*)EEPROM_ALIVE_TOK,alive_tok);
		eeprom_write_byte((uint8_t*)EEPROM_ALIVE_DAY,alive_day);
        eeprom_write_byte((uint8_t*)EEPROM_COM_MODE,com_mode);
        eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
        _delay_ms(2);	// waiting to save.
    } else {
        cali_data = eeprom_read_float((float *)EEPROM_CAL_DATA);
		cali_offset = eeprom_read_byte((uint8_t*)EEPROM_CAL_OFFSET);
		alive_tok = eeprom_read_dword((uint32_t*)EEPROM_ALIVE_TOK);
		alive_day = eeprom_read_byte((uint8_t*)EEPROM_ALIVE_DAY);
        com_mode = eeprom_read_byte((uint8_t*)EEPROM_COM_MODE);
    }
    
    if(com_mode == COM_CALIBRATION) { com_mode = COM_AUTO; }
	if(com_mode == COM_RESET) { com_mode = COM_AUTO; }
    
    led_blink_counter = 5; //led 깜빡임 시작
    while( 1 ) {
        if( led_blink_counter == 0 ) break;
		sprintf(command0,"CLEAN_UP!_BOOT...Ver32\r\n"); TX0_STR(command0); //테스트용
        _delay_ms(1);
    }

    while( 1 ) {
        if( check_bit(PIND,2) == 0 ) { //버튼을 누르면
            _delay_ms(150); //채터링 방지
            button_click = 0;
            
            while( 1 ) {
                _delay_ms(100);//버튼이 2초이상 누르거나 짧게 누르면 빠져 나가기
                if( check_bit(PIND,2) >= 1 ) { break; }
                else { button_click++; }
                //sprintf(command0,"%d,%d,bt\r\n",button_click,check_bit(PIND,2)); TX0_STR(command0); //테스트용
            }
            led_blink_counter = 5;
            if( button_click >= 30 ) {
                led_blink_counter = 10;
                com_mode = COM_CALIBRATION;
            } else {
                if( ++com_mode >= COM_CALIBRATION ) {
                    com_mode = COM_AUTO;
                }
            }
            eep_write_enable = ON;
            _delay_ms(2); //저장시간 딜레이    
        }

        if( com_mode == COM_CALIBRATION ) {
			OCR1A = 0;	// FAN Off.
			cali_data = DEFAULT_DUST_CALIBRATION;
			while( 1 ) {
				dustVout = getAvgDustVoltage();
				cali_data = dustVout;
				if( led_blink_counter == 0 ) {
					eep_write_enable = ON;
					break;
				}
			}
			dust_grade = 0;		// 3 -> 0
			_delay_ms(3000);
			com_mode = COM_AUTO;
        } else {
			if( com_mode == COM_AUTO ) {
				OCR1A = fan_speed_db[dust_grade]; 
			} else if( com_mode == COM_FAN_1 ) { 
				OCR1A = fan_speed_db[FAN_SPEED_NORMAL]; 
			} else if( com_mode == COM_FAN_2 ) { 
				OCR1A = fan_speed_db[FAN_SPEED_BAD]; 
			} else if( com_mode == COM_FAN_3 ) { 
				OCR1A = fan_speed_db[FAN_SPEED_MAX]; 				
			}

			pm_density = getDustDensity();     

			if( pm_density > 0 ) {
				//미세먼지 기준 좋음: 0~15ug/m3 / 보통 16 ~ 35ug/m3 / 나쁨 36이상 / 매우나쁨 76 이상.
				if( pm_density <= DUST_GRADE_GOOD ) {
					dust_grade = 0;
				} else if( pm_density > DUST_GRADE_GOOD && pm_density <= DUST_GRADE_NORMAL ) {
					dust_grade = 1;
				} else if( pm_density > DUST_GRADE_NORMAL && pm_density <= DUST_GRADE_BAD ) {
					dust_grade = 2;
				} else {
					dust_grade = 3;
				}
			} else {
				pm_density = 0;
			}
			alive_tok++;
            sprintf(command0,"%d,%d,%d,%d,%.3f,%.3f,%d,%d,%ld,0\r\n",com_mode,led_light,dust_grade,pm_density,dustVout,cali_data,cali_offset,alive_day,alive_tok); TX0_STR(command0);
		}
      
		if( eep_write_enable == ON ) {
			eeprom_write_byte((uint8_t*)EEPROM_COM_MODE,com_mode);
			eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
			eeprom_write_byte((uint8_t*)EEPROM_CAL_OFFSET, cali_offset);
			eeprom_write_dword((uint32_t*)EEPROM_ALIVE_TOK,alive_tok);
			eeprom_write_byte((uint8_t*)EEPROM_ALIVE_DAY,alive_day);
			eeprom_write_float((float*)EEPROM_CAL_DATA, cali_data);
			eep_write_enable = OFF;
		}      
	}
}
