/**********************************************************

Author: Logan Ngo
Modify: Logan

***********************************************************/

#ifndef DHT11_H
#define DHT11_H

#include "stm32f4xx_hal.h"

#define u32 uint32_t
#define u8 uint8_t

typedef struct{
	u8 temp1;
	u8 temp2;
	u8 hum1;
	u8 hum2;
	u8 check_sum;
}logan;

extern logan dht11;

void delay_us(uint32_t time);
void delay_ms(uint32_t time);
void set_pin_dht_in(void);
void set_pin_dht_out(void);
void DHT11_start(void);
uint8_t read_data();
void dht11_completed(void);

#endif