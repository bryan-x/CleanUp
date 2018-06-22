/*
 * ws2812b.h
 *
 * Created: 2016-06-26 ¿ÀÀü 10:48:27
 *  Author: Run
 */ 


#ifndef WS2812B_H_
#define WS2812B_H_


#include <avr/io.h>
#include <util/delay.h>

#define NOP2 asm volatile("nop"::);asm volatile("nop"::)
#define NOP4 NOP2; NOP2;
#define NOP8 NOP2; NOP2; NOP2; NOP2;


#define WS2812_1	PORTB |= 0x01
#define WS2812_0	PORTB &= ~0x01
void byte_out(char d);
void ws2812b_color(char R , char G , char B , char num);
void ws2812b_init();
void ws2812b_blink(char R, char G, char B, char blink_num);

#endif /* WS2812B_H_ */