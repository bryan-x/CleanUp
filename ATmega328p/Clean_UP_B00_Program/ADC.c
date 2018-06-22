#include "ADC.h"

//   ADC_F=(float)ADC_I*  5.0/1023.0; //ADC_F ����� 0~5�� ��� ���� �� ��� �մϴ�.
//   ADC_F=(float)ADC_I* 10.0/1023.0; //ADC_F ����� 0~10���� ��� ���� �� ��� �մϴ�.
//   ADC_F=(float)ADC_I* 50.0/1023.0; //ADC_F ����� 0~50���� ��� ���� �� ��� �մϴ�.
//   ADC_F=(float)ADC_I*100.0/1023.0; //ADC_F ����� 0~100���� ��� ���� �� ��� �մϴ�.


void ADC_init(){
	ADMUX=0X40; ADCSRA=0xE7; // ADC0���,��������
	
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
char battery_status(){   //���� ��붧���� �̻���
	char i,battery_v=0;  //��Ʈ ����
	char v=0;
	
	//for(i=0;i<4;i++){	v[i]=0;	}
	v=ADC_read(1);  //���͸� ���� ����
	battery_v=4;  //���͸� FULL �� 4
	
	if(v<=86){  //���͸� 13V ~ 11.5V 
		battery_v--;		
		if(v<=75){   //���͸� 11.5V ~ 10V 
			battery_v--;
			if(v<=72){   //���͸� 10V ~ 9.6V 
				battery_v--;	
				if(v<=70){    //���͸� 9.6V ���� 
					battery_v--;	
					
				}
			}	
		}
	}
	//-------------------
	//���͸� 4ĭ= 1 / ���͸� 3ĭ = 2 / ���͸� 2ĭ = 3 / ���͸� 1ĭ = 4
	
	return battery_v;
	
}
*/