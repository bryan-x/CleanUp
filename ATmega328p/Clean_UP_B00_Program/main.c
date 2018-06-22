//DIY ����û����: Clean UP! Program �ҽ�
//Ver B0.0  , ����: SED(SeMin DIY)
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
#define LED_NUM 4
#define SENSOR_LED_ON clr_bit(PORTD,7)
#define SENSOR_LED_OFF set_bit(PORTD,7)
#define MODE_SENSOR 0 //�ڵ� ����
#define MODE_SETUP 4 //Ķ���극�̼ǿ� ���
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
float smoothADC = 0.0; //�ӽ�


unsigned char button_click=0; //��ư Ŭ�� �ð� üũ
uint8_t run_mode; //����û���� ���
uint16_t eep_dust_clean;
float dust_clean_db; //Clean ������ ���ذ�
//--------------------
unsigned int fan_speed_db[3]={FAN_SPEED_LOW,FAN_SPEED_MIDDILE,FAN_SPEED_HI};


unsigned char timer_run=0;
unsigned char led_blink_counter=0; //led ��� ��������

unsigned char i=0;
unsigned char timer_counter=0;
// ��� ������ �ڵ�:��ȫ / �ӵ�1: ��� / �ӵ�2: �Ķ� / �ӵ�3: ���� / �¾�: �����

unsigned char setup_color[5][3]={ {255,0,221}, {0,255,0}, {0,0,255}, {255,0,0}, {255,187,0} };
unsigned char sensor_color[4][3]={ {0,255,0}, {255,187,0}, {255,0,0},{255,187,0}}; //�� �������� ����� , Ķ����
unsigned char led_color_buffer[3];
unsigned char led=0;			
 //---------------------------------- �Ʒ��� EEPROM
char eep_data=0;
float eep_test=0;
  
//----------------------------------
ISR(TIMER0_COMPA_vect){ //LED ������
	if(++timer_counter == 20){ // 5ms x 70 = 350ms	
		if(!(led_blink_counter ==0)){ //0�� �ƴ϶��
			if(++led==2){
				ws2812b_color(0,0,0,4);
				led=0;
			}
			else{
				for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[run_mode][i] * set_color_light; } //��� ����
				ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
				led_blink_counter--; //����
			} 
		}
		//------------------
		else{ //�������� �ƴ϶��
			for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_chart][i] * set_color_light; } //��� ����
			ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
		}
	timer_counter=0;
	}
	
}

int ADC_Dust(){
	static int  dust_mgm3 =0,i,SUM,AVG=0; //���� ��ȯ
	
	SENSOR_LED_ON; //LED Ű��
	_delay_us(280);
	SUM=0;
	for(i=0;i<64;i++){ ADCSRA=0xD7; while(!(ADCSRA&10)); SUM+=ADCW; }
	AVG=SUM>>6;

	_delay_us(39);
	SENSOR_LED_OFF; //����
	_delay_ms(10);
	smoothADC = (AVG*5.0/1023.0) * 0.05 + smoothADC * 0.95;
	dust_mgm3 = (smoothADC - dust_clean_db) / 0.005;
		//sprintf(command0,"%f,%d,%d\r\n",smoothADC,dust_mgm3,test); TX0_STR(command0); //�׽�Ʈ��

	return dust_mgm3;
}


int main(void){
	// ** Port Number: PD7=Sensor LED / PC0(ADC0) = Sensor INPUT / PB1(OC1A)=FAN / PD2(INT0) = BUTTON / PB0 = WS2812B 
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
	
	SREG=0x80; // ���ͷ�Ʈ Ȱ��ȭ
	
	//����: 320ns ���� led�� on �Ǿ�� �ϰ� 280ns ������ ������ ADC �����ؾߵȴ�.
	//up/m3 ���� �̼����� ��[��g/m��] = (Vo ? Voc) / 0.005;
	//�̼����� ���� ����: 0~15ug/m3 / ���� 16 ~ 35ug/m3 / ���� 36�̻�
	
	run_mode = eeprom_read_byte((uint8_t*)EEPROM_RUN_MODE);     // 2���� ���� 1byte read; //eeprom �� �ִ� �ڷḦ �޾ƿ���
	
	if(run_mode>=5){ //run ���� 10���� ũ�� �� ISP �� ó�� ������ EEPROM ���� 255�̴�. �̶��� ���ؼ� �ʱ�ȭ��
		run_mode=0; //run_mode �ʱ�ȭ
		dust_clean_db = DEFULT_DUST_CLEAN; //Clean �� �ʱ�ȭ
		
		eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //���µ� �� �ֱ�
		eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
		_delay_ms(2); //����ð� ������		
		
	}
	else{//���� �����Ͱ� ���µ��� ������
		dust_clean_db= eeprom_read_float((float*)EEPROM_DUST_CLEAN);		
	}
	
	if(run_mode == MODE_SETUP) {run_mode=0;}
	//---------------------------------------�Ʒ��� �ʱ� ����
	
	led_blink_counter=5; //led ������ ����
	while(1){
		if(led_blink_counter==0){break;}
		sprintf(command0,"reset..\r\n"); TX0_STR(command0); //�׽�Ʈ��
	}
	
    while (1) {
		if( check_bit(PIND,2) == 0){ //��ư�� ������
			_delay_ms(150); //ä�͸� ����
			button_click=0;
			
			while(1){
				_delay_ms(100);//��ư�� 2���̻� �����ų� ª�� ������ ���� ������
				if(check_bit(PIND,2)>=1){break;}
				else{button_click++;} //�����ϱ�
				//sprintf(command0,"%d,%d,bt\r\n",button_click,check_bit(PIND,2)); TX0_STR(command0); //�׽�Ʈ��
			}
			led_blink_counter=5;
			if(button_click >= 30){
				run_mode=MODE_SETUP;
			
			} //���� �������
			else {
				if(++run_mode >= 4){
					run_mode=MODE_SENSOR;
				}
			}
			eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);//run �� ����
			_delay_ms(2); //����ð� ������	
			
		}
		//-------------------------------------------------------------
		if(run_mode==MODE_SETUP){ //���� �����
			OCR1A=0; //FAN ����
			
			while(1){
				sprintf(command0,"Senser_SETUP..\r\n"); TX0_STR(command0); //�׽�Ʈ��
				if(led_blink_counter ==0){
					dust_status = ADC_Dust(); //���� �� �ҷ�����...
					dust_clean_db = smoothADC; //���ð� �ֱ�
					eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //���µ� �� �ֱ�
					break;
				} // led ���� �Ÿ������� ��ٸ���
				
			}
			dust_chart = 3; //����� Ű�¿�
			_delay_ms(3000);
			run_mode=0; //�ʱ�ȭ
			
		}
		else{ //�� �� ���
			if(run_mode == MODE_SENSOR){ OCR1A=fan_speed_db[dust_chart]; } //FAN�� ������ ���� �ڵ�����
			else{OCR1A = fan_speed_db[run_mode-1];}//FAN ��� ��������
			
			
			//�Ʒ��� ���� ���� ���� ���α׷�
			dust_status = ADC_Dust(); //���� �� �޾Ƽ� ����
			if(!(dust_status<=0)){ //0���� ������ �ƴ϶��
				//�̼����� ���� ����: 0~15ug/m3 / ���� 16 ~ 35ug/m3 / ���� 36�̻�
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
			else { //0���� ������ ������ üŷ
				dust_status =0;
			}
	
		}
		
		
		sprintf(command0,"%d,%d,%f,%d,%f\r\n",dust_chart,dust_status,smoothADC,run_mode,dust_clean_db); TX0_STR(command0); //�׽�Ʈ��
		//�ܺη� ���: 1.��/��/�� ���� ���� 2. mgm3�� 3.�������� ��µȰ� 4.���� ��� 5. mgm3�� ���ذ� ���� ��
		
		
    }
}



/*
	eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean);
	eeprom_write_byte((unsigned char*)EEPROM_RUN_MODE,6);
	eep_data = eeprom_read_byte((unsigned char*)EEPROM_RUN_MODE);     // 2���� ���� 1byte read
	eep_test = eeprom_read_float((float*)EEPROM_DUST_CLEAN);
	
*/