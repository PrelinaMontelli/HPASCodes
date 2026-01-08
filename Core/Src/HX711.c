#include "HX711.h"
#include "usart.h"
#include "gpio.h"
#include "Filteringalgorithm.h"

static int32_t val = 0;
int pulse_mun=0;
static int32_t HX711_Buffer = 0,First_Weight=0,Weight=0;
static uint8_t filter_initialized;

#ifndef HX711_READY_TIMEOUT_MS
#define HX711_READY_TIMEOUT_MS 50U  /* Max wait for DOUT to go low before aborting */
#endif

#ifndef HX711_READY_TIMEOUT_LOOPS
#define HX711_READY_TIMEOUT_LOOPS 200000UL /* Loop guard in case SysTick is not ticking */
#endif


void HAL_Delay_us(uint32_t delay)
{
	delay=delay*5;
	while(delay)
	{
		delay-=1;
	}
}


long Get_Weight(void)
{
	if (!filter_initialized)
	{
		Filteringalgorithm_Init();
		filter_initialized = 1U;
	}

	HX711_Buffer=Get_number();
	Weight = HX711_Buffer;
	Weight=(long)((float)Weight/103);
	Weight = Filteringalgorithm_Process(Weight);
	return Weight;
}

int32_t Get_number()
{
	val=0;
		  CLK_0;
		  uint32_t start_tick = HAL_GetTick();
		  uint32_t guard = HX711_READY_TIMEOUT_LOOPS;
		  while(!Read_PIN)
		  {
		  	if (((HAL_GetTick() - start_tick) > HX711_READY_TIMEOUT_MS) || (guard-- == 0U))
		  	{
		  		return val; /* Timeout: return last latched value (zero on first call) */
		  	}
		  	__NOP();
		  }
		  for(int i=0;i<24;i++)
		  {
			  HAL_Delay_us(100);
			  CLK_1;
			  val=val<<1;
			  HAL_Delay_us(1);
			  CLK_0;
				HAL_Delay_us(1);
			  if(Read_PIN)
			  val++;
			  HAL_Delay_us(1);

		  }
		  CLK_1;
		  /* sign extend 24-bit two's complement to 32-bit */
		  if (val & 0x800000)
		  {
		  	val |= 0xFF000000;
		  }
		  HAL_Delay_us(1);
		  CLK_0;
		  return val;
		  
}