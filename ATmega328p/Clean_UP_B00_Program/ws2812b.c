/*
 * ws2812b.c
 *
 * Created: 2016-06-26 ���� 10:48:13
 *  Author: Run
 */ 

#include "ws2812b.h"

void ws2812b_color(char R , char G , char B , char num)
{
	char i = 0;	
	for( i = 0; i <num; i++ ) {
		byte_out(G);
		byte_out(R);
		byte_out(B);
	}
	_delay_ms(1);	
}

void ws2812b_init(){
	DDRD=0XFF;
}
/*
void ws2812b_blink(char R, char G, char B, char blink_num){  //Ÿ�̸� ī���� ������ ����ؾ߱� ������ main ������ �̵�
	//4ĭ = ���͸� full , 3ĭ  // 2ĭ // 1ĭ 
	char i;
	
	for(i=0;i<blink_num;i++){
		ws2812b_color(R,G,B,1);
		_delay_ms(150);
		ws2812b_color(0,0,0,1);
		_delay_ms(100);
	}
	
	
	
}*/
void byte_out(char d)
{
	char i;
	for(i=0; i<8; i++) {
		if(d&0x80) {
			WS2812_1; NOP8; WS2812_0; NOP4;
		} else {
			WS2812_1; NOP4; WS2812_0; NOP8;
		}

		d <<= 1;
	}
}
