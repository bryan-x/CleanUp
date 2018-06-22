//DIY ����û����: Clean UP! Program �ҽ�
//MCU: ATMEAG328P / 16MHz / 5V ���
// �����Ϸ�: AVRSTUDIO7
// ===================================================
//Ver v1.1  , ����: SED(SeMin DIY)
//20180606 UPDATE
// ===================================================
//Ver v2.0  , ����: SED(SeMin DIY)
// �߰��ȱ��
// ===================================================
//Ver v3.0  , ����: SED(SeMin DIY)
// �߰��ȱ��
// 1. ����ȭ�� �̼����� ����
// 2. ���� Ķ���극�̼� ��� ���� ���� ����
// 3. ������� ������ ���� ��� ����
//
// ===================================================
// v3.1, ����: Bryan (byoungsu.kr@gmail.com) 2018.06.18
// 1. ���� ���� ��� 3�ܰ� 4�ܰ� ������Ʈ
// 2. ���� ������ ���ø� ���� 30 -> 10 ����
// 3. FAN ���ǵ� 3�ܰ� 4�ܰ� ������Ʈ
// 4. LED ���� ������Ʈ( 4�ܰ� ���� )
// 5. Ķ���극�̼� ������ ���� ������Ʈ(����)
// 6. ����������̽� ���� ����� ���� �ּ�
//
//
//����� ����! , ������ ǥ�� �����Ͽ� ���� �� ���� ����!

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
#define MODE_AUTO 			0
#define MODE_CALIBRATION 	4
#define MODE_FAN_1 			1
#define MODE_FAN_2 			2
#define MODE_FAN_3 			3
 
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
 
#define EEPROM_RUN_MODE 0x30
#define EEPROM_DUST_CALIBRATION 0x00
#define EEPROM_LED_LIGHT 0x40
#define DEFAULT_DUST_CALIBRATION	0.105000
 
#define FAN_SPEED_MAX 		0xFFFF
#define FAN_SPEED_BAD		750
#define FAN_SPEED_NORMAL 	500
#define FAN_SPEED_GOOD 		250
//---------------------------------
#define RX_WRITE_ENABLE 			0xFD
#define RX_WRITE_DISABLE 			0xFB
#define RX_RUN_MODE 		1
#define RX_LED_LIGHT 		2
#define RX_CLEAN_DB_ENABLE 	3
#define RX_CLEAN_DB_DATA 	4
//---------------------------------
#define AVG_DUST_CNT 				10

char command0[30];
char rx_db[10];
unsigned char rx_counter=0;
char rx_write_enable=0;
 
unsigned char dust_grade=0;		// ����(0), ����(1), ����(2), �ſ쳪��(3).
int dust_status=0;			// ���� 0~15ug/m3, ���� 16 ~ 35ug/m3, ���� 36 ~ 75ug/m3, �ſ쳪�� 76�̻�.

unsigned char button_click=0; //��ư Ŭ�� �ð� üũ
uint8_t run_mode; //����û���� ���
uint8_t led_light=0;
float dust_calibration_value = 0.0f;

//--------------------
unsigned int fan_speed_db[4]={FAN_SPEED_GOOD,FAN_SPEED_NORMAL,FAN_SPEED_BAD,FAN_SPEED_MAX};
unsigned char led_blink_counter=0; //led ��� ��������
 
unsigned char timer_counter=0;

// ��� ������ �ڵ�:��ȫ / �ӵ�1: ��� / �ӵ�2: �Ķ� / �ӵ�3: ���� / �¾�: ��Ȳ�� 
unsigned char setup_color[5][3]={ {255,0,221}, {0,255,0}, {0,0,255}, {255,0,0}, {255,187,0} };
unsigned char sensor_color[4][3]={ {0,255,0}, {255,255,0}, {255,187,0},{255,0,0}}; //�� �������� ����� , Ķ����
unsigned char led_color_buffer[3];
unsigned char led=0;            
 //---------------------------------- �Ʒ��� EEPROM

char eep_write_enable = 0;
float dustVout = 0.0f;
//----------------------------------
/*/
���ۺ�Ʈ: 0XFD
����Ʈ: 0XFB
 
1��Ʈ: Run_mode(0~5)
2��Ʈ: LED ������� (0 ~ 10)
3��Ʈ: Ķ���극�̼� ENABLE ��ɾ�
4��Ʈ: Ķ���극�̼� ��(m3ug)
 
*/
 
void calibration_clean_db(int set_m3ug ,  char enable)
{
    //������ ���� ���� ���� Ķ���극�̼� �ϴ� ���
    //���� clean_db �� = ���簪 - (m3ug(��������� ���۹�����) * 0.005);
    
    if(enable == ON){
		dust_calibration_value = dustVout - (set_m3ug * 0.005);
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
		//sprintf(command0,"ok/ %d,%d,%d\r\n",rx_db[0],rx_db[1],rx_db[2]); TX0_STR(command0); //�׽�Ʈ��
		run_mode = rx_db[RX_RUN_MODE];
		led_light = rx_db[RX_LED_LIGHT];

		calibration_clean_db(rx_db[RX_CLEAN_DB_DATA],rx_db[RX_CLEAN_DB_ENABLE]);

		rx_db[RX_CLEAN_DB_ENABLE] = OFF; //�ʱ�ȭ
		eep_write_enable = ON; // EEPROM ���� ��ɾ� Ȱ��ȭ
	}

	if( rx_write_enable == ON ) {
		if( rx_counter == RX_RUN_MODE ) {
			if( !(RX == rx_db[RX_RUN_MODE] )) { //run mode �� �������� ���� �ʴٸ�
				led_blink_counter = 5; //led 5ȸ ��������
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
                for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[run_mode][i] * (led_light * 0.1); } //��� ����
                ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
                led_blink_counter--;
            } 
        } else {
            for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_grade][i] * (led_light * 0.1); } //��� ����
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
		qVoutReadValue[i] = ADCW;

		_delay_us(40);
		SENSOR_LED_OFF;

		_delay_ms(9);		// 9sec Off.
	}

	for ( i = 0; i < AVG_DUST_CNT; i++) {
		sumVoutReadValue = sumVoutReadValue + qVoutReadValue[i];
	}
	avgVoutReadValue = sumVoutReadValue / AVG_DUST_CNT;
	Vo = (float)((avgVoutReadValue * 5.0) / 1024.0);
	
	if( Vo < dust_calibration_value ) {		// Vo �������� Voc ���� ���� ��� ���� ó��.
		dust_calibration_value = Vo;
	}
	return Vo;
}

// u3mg ���ȭ �۾��� ������ ����.
int getDustDensity(void) 
{
	int i = 0, sum_num = AVG_DUST_CNT;
	int sum_dust = 0, dust[AVG_DUST_CNT];
	 
	for( i = 0; i < sum_num; i++ ) {
		dustVout = getAvgDustVoltage();
		dust[i] = (dustVout -  dust_calibration_value) / 0.005;		// ug
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
    uart_init(1); //uart_enable
    ADC_init(); //ADC INIT
    
    TCCR0A= ((1<<WGM01) | (1<<WGM00)); //CTC ��� ���
    TCCR0B= ((1<<CS02) | (0<<CS01) | (1<<CS00)); //1024����
    //001=NO // 010=8 // 011=64 //100=256 // 101=1024
    OCR0A=78;  //5ms
    TIMSK0=((1<<OCIE0A)); 
    
    TCCR1A =0B10000011;
    TCCR1B =0B00001100; //FAST PWM 8����
    OCR1A=0; //�ʱ⿡�� fan ����//FFFF = �� / �� 500 / �� 250 
    
    SREG=0x80;		// Global Interrupt Enable
    
    //����: 320ns ���� led�� on �Ǿ�� �ϰ� 280ns ������ ������ ADC �����ؾߵȴ�.
    //up/m3 ���� �̼����� ��[��g/m��] = (Vo ? Voc) / 0.005;
    //�̼����� ���� ����: 0~15ug/m3 / ���� 16 ~ 35ug/m3 / ���� 36�̻�
    
    led_light = eeprom_read_byte((uint8_t*)EEPROM_LED_LIGHT);
    
    if( led_light > LED_GRADE_MAX || led_light == LED_GRADE_MIN ) {
		// run ���� 10���� ũ�� �� ISP �� ó�� ������ EEPROM ���� 255�̴�. (initialize)
        run_mode = MODE_AUTO;
        dust_calibration_value = DEFAULT_DUST_CALIBRATION;
        led_light = LED_GRADE_5;
        
        eeprom_write_float((float*)EEPROM_DUST_CALIBRATION, dust_calibration_value);
        eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
        eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
        _delay_ms(2);	// waiting to save.
    } else {
        dust_calibration_value = eeprom_read_float((float *)EEPROM_DUST_CALIBRATION);        
        run_mode = eeprom_read_byte((uint8_t*)EEPROM_RUN_MODE);
    }
    
    if(run_mode == MODE_CALIBRATION) { run_mode = MODE_AUTO; }
    
    led_blink_counter=5; //led ������ ����
    while( 1 ) {
        if( led_blink_counter == 0 ) break;
        //sprintf(command0,"CLEAN_UP!_BOOT...Ver30\r\n"); TX0_STR(command0); //�׽�Ʈ��
        _delay_ms(1);
    }

    while( 1 ) {
        if( check_bit(PIND,2) == 0 ) { //��ư�� ������
            _delay_ms(150); //ä�͸� ����
            button_click = 0;
            
            while( 1 ) {
                _delay_ms(100);//��ư�� 2���̻� �����ų� ª�� ������ ���� ������
                if( check_bit(PIND,2) >= 1 ) { break; }
                else { button_click++; }
                //sprintf(command0,"%d,%d,bt\r\n",button_click,check_bit(PIND,2)); TX0_STR(command0); //�׽�Ʈ��
            }
            led_blink_counter = 5;
            if( button_click >= 30 ) {
                led_blink_counter = 10;
                run_mode = MODE_CALIBRATION;
            } else {
                if( ++run_mode >= MODE_CALIBRATION ) {
                    run_mode = MODE_AUTO;
                }
            }
            eep_write_enable = ON;
            _delay_ms(2); //����ð� ������    
        }

        if( run_mode == MODE_CALIBRATION ) {
			OCR1A = 0;	// FAN Off.
			dust_calibration_value = DEFAULT_DUST_CALIBRATION;
			while( 1 ) {
				dustVout = getAvgDustVoltage();
				dust_calibration_value = dustVout;
				//sprintf(command0,"Senser_SETUP..%f\r\n",dustVout); TX0_STR(command0); //�׽�Ʈ��
				if( led_blink_counter == 0 ) {
					eep_write_enable = ON;
					break;
				}
			}
			dust_grade = 3; //����� Ű�¿�
			_delay_ms(3000);
			run_mode = 0;
        } else {
			if( run_mode == MODE_AUTO ) {
				OCR1A = fan_speed_db[dust_grade]; 
			} else { 
				OCR1A = fan_speed_db[run_mode-1]; 
			}

			dust_status = getDustDensity();     

			if( dust_status > 0 ) {
				//�̼����� ���� ����: 0~15ug/m3 / ���� 16 ~ 35ug/m3 / ���� 36�̻� / �ſ쳪�� 76 �̻�.
				if( dust_status <= DUST_GRADE_GOOD ) {
					dust_grade = 0;
				} else if( dust_status > DUST_GRADE_GOOD && dust_status <= DUST_GRADE_NORMAL ) {
					dust_grade = 1;
				} else if( dust_status > DUST_GRADE_NORMAL && dust_status <= DUST_GRADE_BAD ) {
					dust_grade = 2;
				} else {
					dust_grade = 3;
				}
			} else {
				dust_status = 0;
			}         
            sprintf(command0,"%d,%d,%d,%d,%f,%f\r\n",run_mode,led_light,dust_grade,dust_status,dustVout,dust_calibration_value); TX0_STR(command0); //���⿡ ���� ������
		}
      
		if( eep_write_enable == ON ) {
			eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
			eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
			eeprom_write_float((float*)EEPROM_DUST_CALIBRATION,dust_calibration_value);
			eep_write_enable = OFF;
		}      
	}
}
