//DIY 공기청정기: Clean UP! Program 소스
//MCU: ATMEAG328P / 16MHz / 5V 사용
// 컴파일러: AVRSTUDIO7
//Ver v1.1  , 제작: SED(SeMin DIY)
//20180606 UPDATE

//Ver v2.0  , 제작: SED(SeMin DIY)
// 추가된기능
// 1. 디지털 캘리브레이션 기능 추가
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
int dust_status_sum=0; //평균값
char dust_status_temp=0;
float smoothADC = 0.0; //임시
 
unsigned char button_click=0; //버튼 클릭 시간 체크
uint8_t run_mode; //공기청정기 모드
uint8_t led_light=0;
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
char eep_write_enable =0;
  
//----------------------------------
/*/
시작비트: 0XFD
끝비트: 0XFB
 
1비트: Run_mode(0~5)
2비트: LED 밝기제어 (0 ~ 10)
3비트: 캘리브레이션 ENABLE 명령어
4비트: 캘리브레이션 값(m3ug)
 
*/
 
void calibration_clean_db(int set_m3ug ,  char enable){
    //디지털 센서 값을 보고 캘리브레이션 하는 방식
    //공식 clean_db 값 = 현재값 - (m3ug(블루투스로 전송받은값) * 0.005);
    
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
        //sprintf(command0,"ok/ %d,%d,%d\r\n",rx_db[0],rx_db[1],rx_db[2]); TX0_STR(command0); //테스트용
        run_mode=rx_db[RX_RUN_MODE_ADDRESS]; //값 대입
        led_light=rx_db[RX_LED_LIGHT_ADDRESS]; //값 대입
        
        calibration_clean_db(rx_db[RX_CLEAN_DB_DATA_ADDRESS],rx_db[RX_CLEAN_DB_ENABLE_ADDRESS]);
        
        rx_db[RX_CLEAN_DB_ENABLE_ADDRESS] = OFF; //초기화
        eep_write_enable=ON; // EEPROM 저장 명령어 활성화
    }
    
    if(rx_write_enable == ON){
        if(rx_counter==RX_RUN_MODE_ADDRESS){
            if(!(RX == rx_db[RX_RUN_MODE_ADDRESS])){ //run mode 가 기존값과 같지 않다면
                led_blink_counter=5; //led 5회 깜빡여라
            }
        }
        rx_db[rx_counter] = RX;
        rx_counter++;
    }            
}
 
ISR(TIMER0_COMPA_vect){ //LED 깜빡임
    if(++timer_counter == 20){ // 5ms x 70 = 350ms    
        if(!(led_blink_counter ==0)){ //0이 아니라면
            if(++led==2){
                ws2812b_color(0,0,0,4);
                led=0;
            }
            else{
                for(i=0;i<3;i++){  led_color_buffer[i] = setup_color[run_mode][i] * (led_light * 0.1); } //밝기 조절
                ws2812b_color(led_color_buffer[R],led_color_buffer[G],led_color_buffer[B],WS2812B_LED_NUM);
                led_blink_counter--; //빼기
            } 
        }
        //------------------
        else{ //깜빡임이 아니라면
            for(i=0;i<3;i++){  led_color_buffer[i] = sensor_color[dust_chart][i] * (led_light * 0.1); } //밝기 조절
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
    
    led_light = eeprom_read_byte((uint8_t*)EEPROM_LED_LIGHT);
    
    if(led_light>=15||led_light==0){ //run 값이 10보다 크면 즉 ISP 로 처음 넣을때 EEPROM 값은 255이다. 이때를 위해서 초기화용
        run_mode=DEFULT_RUN_MODE; //run_mode 초기화
        dust_clean_db = DEFULT_DUST_CLEAN; //Clean 값 초기화
        led_light = DEFULT_LED_LIGHT;
        
        eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db); //리셋된 값 넣기
        eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);
        eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
        _delay_ms(2); //저장시간 딜레이        
    }
    else{//만약 데이터가 리셋되지 않으면
        dust_clean_db= eeprom_read_float((float*)EEPROM_DUST_CLEAN);        
        run_mode = eeprom_read_byte((uint8_t*)EEPROM_RUN_MODE);     
    }
    
    if(run_mode == MODE_SETUP) {run_mode=0;}
    //---------------------------------------아래는 초기 절차
    
    led_blink_counter=5; //led 깜빡임 시작
    while(1){
        if(led_blink_counter==0){break;}
        sprintf(command0,"CLEAN_UP!_BOOT...Ver11\r\n"); TX0_STR(command0); //테스트용
    }
    //--------------------여기가 시작
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
                led_blink_counter=10;
                run_mode=MODE_SETUP;
            } //센서 측정모드
            else {
                if(++run_mode >= 4){
                    run_mode=MODE_SENSOR;
                }
            }
            eep_write_enable=ON; //eeprom 저장해라
            _delay_ms(2); //저장시간 딜레이    
            
        }
        //-------------------------------------------------------------
        if(run_mode==MODE_SETUP){ //설정 모드라면
            OCR1A=0; //FAN 끄기
            dust_clean_db=DEFULT_DUST_CLEAN; //낮은값 있으면 큰일나니 일단 초기화
            while(1){
            
                //sprintf(command0,"Senser_SETUP.. %f /%f \r\n",dust_clean_db,smoothADC); TX0_STR(command0); //테스트용
                dust_status = ADC_Dust(); //센서 값 불러오기...
                
                if(dust_clean_db>=smoothADC){
                    dust_clean_db=smoothADC; //셋팅값 넣기    
                }
                
                sprintf(command0,"Senser_SETUP..%f\r\n",smoothADC); TX0_STR(command0); //테스트용
                
                if(led_blink_counter ==0){
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
            if(++dust_status_temp==DUST_average){//평균값에 도달하면
                dust_status = dust_status_sum / DUST_average; //평균 구하기    
                
                //아래는 평균값을 기반으로 계산
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
            //------------------------------
                dust_status_sum=0; //초기화
                dust_status_temp=0; //초기화
                sprintf(command0,"%d,%d,%f,%d,%d,%f\r\n",dust_chart,dust_status,smoothADC,run_mode,led_light,dust_clean_db); TX0_STR(command0); //테스트용
                //외부로 출력: 1.상/중/하 먼지 상태 2. mgm3값 3.센서에서 출력된값 4.현재 모드 5. mgm3의 기준값 공기 값
 
            }
            
            else{
                
                dust_status_sum += ADC_Dust(); //센서 값 받아서 저장
                    
            }
            if(eep_write_enable == ON){//RUN 모드는 버튼 + 블루투스에서 2개에서 명령이 내려와서 , 한곳에서 통합 저장할수 있도록함
                eeprom_write_byte((uint8_t*)EEPROM_RUN_MODE,run_mode);//run 값 저장
                eeprom_write_byte((uint8_t*)EEPROM_LED_LIGHT,led_light);
                eeprom_write_float((float*)EEPROM_DUST_CLEAN,dust_clean_db);
                    
                eep_write_enable=OFF; //지가 종료 끄기
            }
    //-------------------------------    
        }
    }
}
