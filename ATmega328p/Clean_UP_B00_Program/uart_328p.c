#include "uart_328p.h"
/*
// 2016-05-07 ���� ������Ʈ
// ���� char command0[30],command1[30]; �� �߰� �ϼ��� (char)
//     sprintf(command0,""); TX0_STR(command0);
//     sprintf(command1,""); TX1_STR(command1);
//		uart_init ���� start_uart0 , start_uart1 , start_uart_all
*/




//

void TX0_CH(char c){ while(!(UCSR0A&0x20)); UDR0=c; } // 1����Ʈ �۽�
void TX0_STR(char *s){ while(*s)TX0_CH(*s++); } // ���ڿ� �۽�


void uart_init(char uart_stats1){
	// 1 = UART0 , 2 = UART=1 / 3 = ALL
	
	switch(uart_stats1){
		case 1:  UCSR0B=0x98; UBRR0L=103;  // GPS 9600
		sei();
		break;

	}
	
	
}