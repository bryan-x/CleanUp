//DIY 공기청정기: Clean UP! Program 소스
//Ver B0.0  , 제작: SED(SeMin DIY)
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
#define LED_NUM 4
#define SENSOR_LED_ON clr_bit(PORTD,7)
#define SENSOR_LED_OFF set_bit(PORTD,7)
#define MODE_SENSOR 0 //자동 동작
#define MODE_SETUP 4 //캘리브레이션용 모드
#define MODE_FAN_1 1
#define MODE_FAN_2 2
#define MODE_FAN_3 3

#define R 0
#define G 1
#define B 2
#define WS2812B_LED_NUM 4

#define set_color_light 0.5
#define set_sensor_light 0.5

#define DUST_MAX 500
#define DUST_HI 100
#define DUST_MIDIUM 60
#define DUST_CLEAN 30

#define EEPROM_RUN_MODE 0X30
#define EEPROM_DUST_CLEAN 0X00
#define DEFULT_DUST_CLEAN 0.295762
#define DEFULT_RUN_MODE 0

#define FAN_SPEED_HI 0XFFFF
#define FAN_SPEED_MIDDILE 500
#define FAN_SPEED_LOW 250
//---------------------------------
char command0[30];
unsigned char dust_chart=0;
int dust_status=0;
float smoothADC = 0.0; //임시


unsigned char button_click=0; //버튼 클릭 시간 체크
uint8_t run_mode; //공기청정기 모드
uint16_t eep_dust_clean;
float dust_clean_db; //Clean 했을때 기준값
//--------------------
unsigned int fan_speed_db[3]={FAN_SPEED_LOW,FAN_SPEED_MIDDILE,FAN_SPEED_HI};


unsigned char timer_run=0;
unsigned char led_blink_counter=0; //led 몇번 깜박일지

unsigned char i=0;
unsigned char timer_counter=0;
// 모드 색기준 자동:분홍 / 속도1: 녹색 / 속도2: 파랑 / 속도3: 빨강 / 셋업: 노란색

unsigned char setup_color[5][3]={ {255,0,221}, {0,255,0}, {0,0,255}, {255,0,0}, {255,187,0} };
unsigned char sensor_color[4][3]={ {0,255,0}, {255,187,0}, {255,0,0},{255,187,0}}; //맨 마지막은 노란색 , 캘리용
unsigned char led_color_buffer[3];
unsigned char led=0;			
 //---------------------------------- 아래는 EEPROM
char eep_data=0;
float eep_test=0;
  
//----------------------------------
ISR(TIMER0_COMPA_vect){ //LED 깜빡임
	if(++timer_counter == 20){ // 5ms x 70 = 350ms	
		if(!(led_blink_counter ==0)){ //0이 아니라면
			if(++led==2){
				ws2812b_color(0,0,0,4);
				led=0;
			}
			else{
				for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[run_mode][i] * set_color_light; } //밝기 조절
				ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
				led_blink_counter--; //빼기
			} 
		}
		//------------------
		else{ //깜빡임이 아니라면
			for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_chart][i] * set_color_light; } //밝기 조절
			ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
		}
	timer_counter=0;
	}
	
}

int ADC_Dust(){
	static int  dust_mgm3 =0,i,SUM,AVG=0; //단위 변환
	
	SENSOR_LED_ON; //LED 키기
	_delay_us(280);
	SUM=0;
	for(i=0;i<64;i++){ ADCSRA=0xD7; while(!(ADCSRA&10)); SUM+=ADCW; }
	AVG=SUM>>6;

	_delay_us(39);
	SENSOR_LED_OFF; //끄기
	_delay_ms(10);
	smoothADC = (AVG*5.0/1023.0) * 0.05 + smoothADC * 0.95;
	dust_mgm3 = (smoothADC - dust_clean_db) / 0.005;
		//sprintf(command0,"%f,%d,%d\r\n",smoothADC,dust_mgm3,test); TX0_STR(command0); //테스트용

	return dust_mgm3;
}


int main(void){
	// ** Port Number: PD7=Sensor LED / PC0(ADC0) = Sensor INPUT / PB1(OC1A)=FAN / PD2(INT0) = BUTTON / PB0 = WS2812B 
	DDRD=0b11111011; DDRB=0XFF;
	uart_init(1); //uart_enable
	ADC_init(); //ADC INIT
	
	TCCR0A= ((1<<WGM01) | (1<<WGM00)); //CTC 모드 사용
	TCCR0B= ((1<<CS02) | (0<<CS01) | (1<<CS00)); //1024분주
	//001=NO // 010=8 // 011=64 //100=256 // 101=1024
	OCR0A=78;  //5ms
	TIMSK0=((1<<OCIE0A)); 
	
	TCCR1A =0B10000011;
	TCCR1B =0B00001100; //FAST PWM 8분주
	OCR1A=0; //초기에는 fan 끄기//FFFF = 강 / 중 500 / 약 250 
	
	SREG=0x80; // 인터럽트 활성화
	
	//원리: 320ns 마다 led가 on 되어야 하고 280ns 지나는 시점에 ADC 센싱해야된다.
	//up/m3 공식 미세먼지 농도[μg/m³] = (Vo ? Voc) / 0.005;
	//미세먼지 기준 좋음: 0~15ug/m3 / 보통 16 ~ 35ug/m3 / 나쁨 36이상
	
	run_mode = eeprom_read_byte((uint8_t*)EEPROM_RUN_MODE);     // 2번지 에서 1byte read; //eeprom 에 있는 자료를 받아오기
	
	if(run_mode>=5){ //run 값이 10보다 크면 즉 ISP 로 처음 넣을때 EEPROM 값은 255이다. 이때를 위해서 초기화용
		run_mode=0; //run_mode 초기화
		dust_clean_db = DEFULT_DUST_CLEAN; //Clean 값 초기화
		
		eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //리셋된 값 넣기
		eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
		_delay_ms(2); //저장시간 딜레이		
		
	}
	else{//만약 데이터가 리셋되지 않으면
		dust_clean_db= eeprom_read_float((float*)EEPROM_DUST_CLEAN);		
	}
	
	if(run_mode == MODE_SETUP) {run_mode=0;}
	//---------------------------------------아래는 초기 절차
	
	led_blink_counter=5; //led 깜빡임 시작
	while(1){
		if(led_blink_counter==0){break;}
		sprintf(command0,"reset..\r\n"); TX0_STR(command0); //테스트용
	}
	
    while (1) {
		if( check_bit(PIND,2) == 0){ //버튼을 누르면
			_delay_ms(150); //채터링 방지
			button_click=0;
			
			while(1){
				_delay_ms(100);//버튼이 2초이상 누르거나 짧게 누르면 빠져 나가기
				if(check_bit(PIND,2)>=1){break;}
				else{button_click++;} //증가하기
				//sprintf(command0,"%d,%d,bt\r\n",button_click,check_bit(PIND,2)); TX0_STR(command0); //테스트용
			}
			led_blink_counter=5;
			if(button_click >= 30){
				run_mode=MODE_SETUP;
			
			} //센서 측정모드
			else {
				if(++run_mode >= 4){
					run_mode=MODE_SENSOR;
				}
			}
			eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);//run 값 저장
			_delay_ms(2); //저장시간 딜레이	
			
		}
		//-------------------------------------------------------------
		if(run_mode==MODE_SETUP){ //설정 모드라면
			OCR1A=0; //FAN 끄기
			
			while(1){
				sprintf(command0,"Senser_SETUP..\r\n"); TX0_STR(command0); //테스트용
				if(led_blink_counter ==0){
					dust_status = ADC_Dust(); //센서 값 불러오기...
					dust_clean_db = smoothADC; //셋팅값 넣기
					eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //리셋된 값 넣기
					break;
				} // led 깜빡 거릴때까지 기다린다
				
			}
			dust_chart = 3; //노란색 키는용
			_delay_ms(3000);
			run_mode=0; //초기화
			
		}
		else{ //그 외 모드
			if(run_mode == MODE_SENSOR){ OCR1A=fan_speed_db[dust_chart]; } //FAN이 먼지에 따라 자동제어
			else{OCR1A = fan_speed_db[run_mode-1];}//FAN 모드 수동제어
			
			
			//아래는 먼지 측정 센서 프로그램
			dust_status = ADC_Dust(); //센서 값 받아서 저장
			if(!(dust_status<=0)){ //0보다 작은게 아니라면
				//미세먼지 기준 좋음: 0~15ug/m3 / 보통 16 ~ 35ug/m3 / 나쁨 36이상
				if(dust_status < DUST_MAX){
					dust_chart=2;
					if(dust_status < DUST_MIDIUM){
						dust_chart=1;
						if(dust_status < DUST_CLEAN){
							dust_chart=0;
						}
					}
				
				}
			
			}
			else { //0보다 작으면 오류로 체킹
				dust_status =0;
			}
	
		}
		
		
		sprintf(command0,"%d,%d,%f,%d,%f\r\n",dust_chart,dust_status,smoothADC,run_mode,dust_clean_db); TX0_STR(command0); //테스트용
		//외부로 출력: 1.상/중/하 먼지 상태 2. mgm3값 3.센서에서 출력된값 4.현재 모드 5. mgm3의 기준값 공기 값
		
		
    }
}



/*
	eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean);
	eeprom_write_byte((unsigned char*)EEPROM_RUN_MODE,6);
	eep_data = eeprom_read_byte((unsigned char*)EEPROM_RUN_MODE);     // 2번지 에서 1byte read
	eep_test = eeprom_read_float((float*)EEPROM_DUST_CLEAN);
	
*/