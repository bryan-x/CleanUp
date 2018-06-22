/*
 * uart_328p.h
 *
 * Created: 2016-06-16 ���� 7:00:01
 *  Author: Run
 */ 


#ifndef UART_328P_H_
#define UART_328P_H_


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#define start_uart 1



void uart_init(char uart_stats1);
void TX0_CH(char c); // 1����Ʈ �۽�
void TX0_STR(char *s); // ���ڿ� �۽�
void ws2812b_battery(); //ws2812b ���͸�






#endif /* UART_328P_H_ */