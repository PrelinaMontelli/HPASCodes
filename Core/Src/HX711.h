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

/* 实验性事件标志位（仅在 HX711_EXPERIMENTAL=1 时有效） */
#define HX711_EXP_EVT_STARTUP_TARE 0x01U
#define HX711_EXP_EVT_AUTOZERO     0x02U

#ifndef HX711_EXPERIMENTAL
#define HX711_EXPERIMENTAL 1 /* 总开关：1 开启实验性功能，0 关闭 */
#endif

void HAL_Delay_us(uint32_t delay);
int32_t Get_number(void);
long Get_Weight(void);
uint32_t HX711_GetAndClearExpEvents(void);

#endif
