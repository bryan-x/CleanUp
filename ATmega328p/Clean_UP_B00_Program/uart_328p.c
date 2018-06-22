#include "uart_328p.h"
/*
// 2016-05-07 최종 업데이트
// 사용시 char command0[30],command1[30]; 를 추가 하세요 (char)
//     sprintf(command0,""); TX0_STR(command0);
//     sprintf(command1,""); TX1_STR(command1);
//		uart_init 사용시 start_uart0 , start_uart1 , start_uart_all
*/




//

void TX0_CH(char c){ while(!(UCSR0A&0x20)); UDR0=c; } // 1바이트 송신
void TX0_STR(char *s){ while(*s)TX0_CH(*s++); } // 문자열 송신


void uart_init(char uart_stats1){
	// 1 = UART0 , 2 = UART=1 / 3 = ALL
	
	switch(uart_stats1){
		case 1:  UCSR0B=0x98; UBRR0L=103;  // GPS 9600
		sei();
		break;

	}
	
	
}