//DIY ����û����: Clean UP! Program �ҽ�
//MCU: ATMEAG328P / 16MHz / 5V ���
// �����Ϸ�: AVRSTUDIO7
//Ver v1.1  , ����: SED(SeMin DIY)
//20180606 UPDATE

//Ver v2.0  , ����: SED(SeMin DIY)
// �߰��ȱ��
// 1. ������ Ķ���극�̼� ��� �߰�
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
 
 
#define DUST_MAX 500
#define DUST_HI 500
#define DUST_MIDIUM 50
#define DUST_CLEAN 15
 
#define EEPROM_RUN_MODE 0X30
#define EEPROM_DUST_CLEAN 0X00
#define EEPROM_LED_LIGHT 0X40
#define DEFULT_DUST_CLEAN 0.105000
#define DEFULT_RUN_MODE 0
#define DEFULT_LED_LIGHT 5
#define DUST_average 40
 
#define FAN_SPEED_HI 0XFFFF
#define FAN_SPEED_MIDDILE 500
#define FAN_SPEED_LOW 250
//---------------------------------
#define RX_WRITE_ENABLE 0XFD
#define RX_WRITE_DISABLE 0XFB
#define RX_RUN_MODE_ADDRESS 1
#define RX_LED_LIGHT_ADDRESS 2
#define RX_CLEAN_DB_ENABLE_ADDRESS 3
#define RX_CLEAN_DB_DATA_ADDRESS 4
 
char command0[30];
char rx_db[10];
char rx_counter=0;
char rx_write_enable=0;
 
unsigned char dust_chart=0;
int dust_status=0;
int dust_status_sum=0; //��հ�
char dust_status_temp=0;
float smoothADC = 0.0; //�ӽ�
 
unsigned char button_click=0; //��ư Ŭ�� �ð� üũ
uint8_t run_mode; //����û���� ���
uint8_t led_light=0;
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
char eep_write_enable =0;
  
//----------------------------------
/*/
���ۺ�Ʈ: 0XFD
����Ʈ: 0XFB
 
1��Ʈ: Run_mode(0~5)
2��Ʈ: LED ������� (0 ~ 10)
3��Ʈ: Ķ���극�̼� ENABLE ��ɾ�
4��Ʈ: Ķ���극�̼� ��(m3ug)
 
*/
 
void calibration_clean_db(int set_m3ug ,  char enable){
    //������ ���� ���� ���� Ķ���극�̼� �ϴ� ���
    //���� clean_db �� = ���簪 - (m3ug(��������� ���۹�����) * 0.005);
    
    if(enable == ON){
        dust_clean_db = smoothADC - (set_m3ug * 0.005);
    }
    
}
 
ISR(USART_RX_vect){
    char RX;
    RX=UDR0;
 
    if(RX == RX_WRITE_ENABLE){rx_write_enable = ON;}
    else if(RX == RX_WRITE_DISABLE){
        rx_write_enable=OFF; 
        rx_counter=0;
        //sprintf(command0,"ok/ %d,%d,%d\r\n",rx_db[0],rx_db[1],rx_db[2]); TX0_STR(command0); //�׽�Ʈ��
        run_mode=rx_db[RX_RUN_MODE_ADDRESS]; //�� ����
        led_light=rx_db[RX_LED_LIGHT_ADDRESS]; //�� ����
        
        calibration_clean_db(rx_db[RX_CLEAN_DB_DATA_ADDRESS],rx_db[RX_CLEAN_DB_ENABLE_ADDRESS]);
        
        rx_db[RX_CLEAN_DB_ENABLE_ADDRESS] = OFF; //�ʱ�ȭ
        eep_write_enable=ON; // EEPROM ���� ��ɾ� Ȱ��ȭ
    }
    
    if(rx_write_enable == ON){
        if(rx_counter==RX_RUN_MODE_ADDRESS){
            if(!(RX == rx_db[RX_RUN_MODE_ADDRESS])){ //run mode �� �������� ���� �ʴٸ�
                led_blink_counter=5; //led 5ȸ ��������
            }
        }
        rx_db[rx_counter] = RX;
        rx_counter++;
    }            
}
 
ISR(TIMER0_COMPA_vect){ //LED ������
    if(++timer_counter == 20){ // 5ms x 70 = 350ms    
        if(!(led_blink_counter ==0)){ //0�� �ƴ϶��
            if(++led==2){
                ws2812b_color(0,0,0,4);
                led=0;
            }
            else{
                for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[run_mode][i] * (led_light * 0.1); } //��� ����
                ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
                led_blink_counter--; //����
            } 
        }
        //------------------
        else{ //�������� �ƴ϶��
            for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_chart][i] * (led_light * 0.1); } //��� ����
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
    
    led_light = eeprom_read_byte((uint8_t*)EEPROM_LED_LIGHT);
    
    if(led_light>=15||led_light==0){ //run ���� 10���� ũ�� �� ISP �� ó�� ������ EEPROM ���� 255�̴�. �̶��� ���ؼ� �ʱ�ȭ��
        run_mode=DEFULT_RUN_MODE; //run_mode �ʱ�ȭ
        dust_clean_db = DEFULT_DUST_CLEAN; //Clean �� �ʱ�ȭ
        led_light = DEFULT_LED_LIGHT;
        
        eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //���µ� �� �ֱ�
        eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
        eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
        _delay_ms(2); //����ð� ������        
    }
    else{//���� �����Ͱ� ���µ��� ������
        dust_clean_db= eeprom_read_float((float*)EEPROM_DUST_CLEAN);        
        run_mode = eeprom_read_byte((uint8_t*)EEPROM_RUN_MODE);     
    }
    
    if(run_mode == MODE_SETUP) {run_mode=0;}
    //---------------------------------------�Ʒ��� �ʱ� ����
    
    led_blink_counter=5; //led ������ ����
    while(1){
        if(led_blink_counter==0){break;}
        sprintf(command0,"CLEAN_UP!_BOOT...Ver11\r\n"); TX0_STR(command0); //�׽�Ʈ��
    }
    //--------------------���Ⱑ ����
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
                led_blink_counter=10;
                run_mode=MODE_SETUP;
            } //���� �������
            else {
                if(++run_mode >= 4){
                    run_mode=MODE_SENSOR;
                }
            }
            eep_write_enable=ON; //eeprom �����ض�
            _delay_ms(2); //����ð� ������    
            
        }
        //-------------------------------------------------------------
        if(run_mode==MODE_SETUP){ //���� �����
            OCR1A=0; //FAN ����
            dust_clean_db=DEFULT_DUST_CLEAN; //������ ������ ū�ϳ��� �ϴ� �ʱ�ȭ
            while(1){
            
                //sprintf(command0,"Senser_SETUP.. %f /%f \r\n",dust_clean_db,smoothADC); TX0_STR(command0); //�׽�Ʈ��
                dust_status = ADC_Dust(); //���� �� �ҷ�����...
                
                if(dust_clean_db>=smoothADC){
                    dust_clean_db=smoothADC; //���ð� �ֱ�    
                }
                
                sprintf(command0,"Senser_SETUP..%f\r\n",smoothADC); TX0_STR(command0); //�׽�Ʈ��
                
                if(led_blink_counter ==0){
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
            if(++dust_status_temp==DUST_average){//��հ��� �����ϸ�
                dust_status = dust_status_sum / DUST_average; //��� ���ϱ�    
                
                //�Ʒ��� ��հ��� ������� ���
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
            //------------------------------
                dust_status_sum=0; //�ʱ�ȭ
                dust_status_temp=0; //�ʱ�ȭ
                sprintf(command0,"%d,%d,%f,%d,%d,%f\r\n",dust_chart,dust_status,smoothADC,run_mode,led_light,dust_clean_db); TX0_STR(command0); //�׽�Ʈ��
                //�ܺη� ���: 1.��/��/�� ���� ���� 2. mgm3�� 3.�������� ��µȰ� 4.���� ��� 5. mgm3�� ���ذ� ���� ��
 
            }
            
            else{
                
                dust_status_sum += ADC_Dust(); //���� �� �޾Ƽ� ����
                    
            }
            if(eep_write_enable == ON){//RUN ���� ��ư + ����������� 2������ ����� �����ͼ� , �Ѱ����� ���� �����Ҽ� �ֵ�����
                eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);//run �� ����
                eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
                eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db);
                    
                eep_write_enable=OFF; //���� ���� ����
            }
    //-------------------------------    
        }
    }
}
