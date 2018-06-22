#include "ADC.h"

//   ADC_F=(float)ADC_I*  5.0/1023.0; //ADC_F 결과를 0~5로 얻고 싶을 때 사용 합니다.
//   ADC_F=(float)ADC_I* 10.0/1023.0; //ADC_F 결과를 0~10으로 얻고 싶을 때 사용 합니다.
//   ADC_F=(float)ADC_I* 50.0/1023.0; //ADC_F 결과를 0~50으로 얻고 싶을 때 사용 합니다.
//   ADC_F=(float)ADC_I*100.0/1023.0; //ADC_F 결과를 0~100으로 얻고 싶을 때 사용 합니다.


void ADC_init(){
	ADMUX=0X40; ADCSRA=0xE7; // ADC0사용,프리러닝
	
}

void delay_us(unsigned char time_us) // time delay(us)
{
	register unsigned char i;
	for(i = 0; i < time_us; i++) {  // 4 cycle +
		asm volatile(" PUSH R0 "); // 2 cycle +
		asm volatile(" POP R0 ");  // 2 cycle +
		asm volatile(" PUSH R0 "); // 2 cycle +
		asm volatile(" POP R0 ");  // 2 cycle +
		asm volatile(" PUSH R0 "); // 2 cycle +
		asm volatile(" POP R0 ");  // 2 cycle   16 cycle = 1.085 us for 14.745600MHz
	}
}


float ADC_read()
{
	int ADC_I;
	//float ADC_F;
	
	ADC_I = ADCW;
	
	//ADC_F=(float)ADC_I*180.0/1023.0; 
	
	return ADC_I;
}
/*
char battery_status(){   //변수 사용때문에 이사함
	char i,battery_v=0;  //볼트 상태
	char v=0;
	
	//for(i=0;i<4;i++){	v[i]=0;	}
	v=ADC_read(1);  //배터리 상태 읽음
	battery_v=4;  //배터리 FULL 시 4
	
	if(v<=86){  //배터리 13V ~ 11.5V 
		battery_v--;		
		if(v<=75){   //배터리 11.5V ~ 10V 
			battery_v--;
			if(v<=72){   //배터리 10V ~ 9.6V 
				battery_v--;	
				if(v<=70){    //배터리 9.6V 이하 
					battery_v--;	
					
				}
			}	
		}
	}
	//-------------------
	//배터리 4칸= 1 / 배터리 3칸 = 2 / 배터리 2칸 = 3 / 배터리 1칸 = 4
	
	return battery_v;
	
}
*/