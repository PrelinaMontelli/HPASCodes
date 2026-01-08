#ifndef _HX711_H_
#define _HX711_H_
#include "main.h"
#include <stdint.h>

#define CLK_1 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,1)
#define CLK_0 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,0)
#define Read_PIN HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)
#define Gain_128 25
#define Gain_64 27
#define Gain_32 26
void HAL_Delay_us(uint32_t delay);
int32_t Get_number(void);
long Get_Weight(void);

#endif