/**********************************************************

Author: Logan Ngo
Modify: Logan

***********************************************************/

#include "hcsr05.h"

extern TIM_HandleTypeDef htim2;

float distance=0.0;

void SR05_Init(void){
	HAL_TIM_Base_Start(&htim2);
}

void SR05_Trigger(void){
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, 1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRIGGER_PORT, TRIGGER_PIN, 0);
}

float SR05_Active(void){
	uint16_t time;
	float dis;
	SR05_Trigger();
	while(!HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN));
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN));
	time = __HAL_TIM_GET_COUNTER(&htim2);
	dis = (time*1.0)*(0.034/2);
	return dis;
}

void STOP_DISTANCE(void){
	if(distance <= 3.0){

	}
}

