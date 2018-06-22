
#ifndef ADC_H_
#define ADC_H_

#include<avr/io.h>
#include<util/delay.h>

void ADC_init();
float ADC_read();
char battery_status();
void delay_us(unsigned char time_us);




#endif /* ADC_H_ */