#include "dht11.h"
extern TIM_HandleTypeDef htim1;

#define u32 uint32_t
#define u8 uint8_t

extern logan dht11;

void delay_us(u32 time){
	HAL_TIM_Base_Start(&htim1);
	htim1.Instance->CNT=0;
	while(htim1.Instance->CNT<time);
	HAL_TIM_Base_Stop(&htim1);
}
void delay_ms(u32 time){
	for(u32 i=0; i<1000; i++){
		delay_us(1);
	}
}

void set_pin_dht_in(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void set_pin_dht_out(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void DHT11_start(void){
	set_pin_dht_out();
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,0); //Dua tin hieu xg muc thap
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,1);
	set_pin_dht_in();
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0);
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1);
}

u8 read_data ()
{
	u8 i=0,j;
	for (j=0;j<8;j++)
	{
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==0);
		delay_us(45);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1){
			i = (i<<1) | (1<<0);
		}else{
			i = (i<<1) & ~(1<<0);
		}
		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)==1);
	}
	return i;
}

void dht11_completed(){
	DHT11_start();
	dht11.hum1 = read_data();
	dht11.hum2 = read_data();
	dht11.temp1 = read_data();
	dht11.temp2 = read_data();
	dht11.check_sum = read_data();
}