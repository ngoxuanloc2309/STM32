/**********************************************************

Author: Logan Ngo
Modify: Logan

***********************************************************/

#ifndef _HCSR05_H
#define _HCSR05_H
#include "stm32f4xx_hal.h"

#define TRIGGER_PORT 	GPIOB
#define TRIGGER_PIN		GPIO_PIN_9

#define ECHO_PORT		GPIOB
#define ECHO_PIN		GPIO_PIN_8


void SR05_Init(void);
void SR05_Trigger(void);
float SR05_Active(void);
void STOP_DISTANCE(void);

#endif