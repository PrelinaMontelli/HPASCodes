#include "HX711.h"
#include "usart.h"
#include "gpio.h"
#include "Filteringalgorithm.h"

/* ===================== 实验性功能区域（通过宏开关） ===================== */


#if HX711_EXPERIMENTAL
/* 各实验功能的开关 */
#ifndef HX711_EXP_ENABLE_ZERO_TRACK
#define HX711_EXP_ENABLE_ZERO_TRACK 1 /* 零点跟踪：稳定且近零时允许零点微调 */
#endif
#ifndef HX711_EXP_ENABLE_STARTUP_TARE
#define HX711_EXP_ENABLE_STARTUP_TARE 1 /* 开机稳定去皮：上电稳定后记录零点 */
#endif
#ifndef HX711_EXP_ENABLE_AUTO_ZERO
#define HX711_EXP_ENABLE_AUTO_ZERO 1 /* 自动零点修正：近零窗口内小步长归零 */
#endif

/* 可调参数（单位为原始计数，线性化前） */
#ifndef HX711_EXP_WINDOW_SIZE
#define HX711_EXP_WINDOW_SIZE 16U /* 稳定检测窗口大小（样本数） */
#endif
#ifndef HX711_EXP_STABLE_SPAN
#define HX711_EXP_STABLE_SPAN 5L /* 稳定判定的峰峰值阈值 */
#endif
#ifndef HX711_EXP_STABLE_TIME_MS
#define HX711_EXP_STABLE_TIME_MS 800U /* 稳定后需要持续的时间 */
#endif
#ifndef HX711_EXP_AUTOZERO_WINDOW
#define HX711_EXP_AUTOZERO_WINDOW 10L /* 自动归零允许的近零窗口 */
#endif
#ifndef HX711_EXP_AUTOZERO_STEP
#define HX711_EXP_AUTOZERO_STEP 1L /* 每次归零调整的步长 */
#endif
#ifndef HX711_EXP_ZERO_DRIFT_LIMIT
#define HX711_EXP_ZERO_DRIFT_LIMIT 500L /* 允许的零点累计漂移上限 */
#endif
#ifndef HX711_EXP_STARTUP_SETTLE_MS
#define HX711_EXP_STARTUP_SETTLE_MS 1500U /* 上电后等待稳定再去皮的时间 */
#endif

typedef struct
{
	long buf[HX711_EXP_WINDOW_SIZE];
	uint8_t idx;
	uint8_t count;
	long sum;
	long minv;
	long maxv;
	long zero_offset;
	long drift_accum;
	uint8_t startup_tared;
	uint32_t stable_since;
	uint32_t start_tick;
} hx711_exp_state_t;

static hx711_exp_state_t g_exp_state;
static volatile uint32_t g_exp_events;
static long HX711_Experimental_Process(long sample);
#endif /* HX711_EXPERIMENTAL */
/* =================== 实验性功能区域结束（通过宏开关） =================== */

static int32_t val = 0;
static int32_t last_valid = 0;
static uint8_t has_valid_sample = 0U;
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
#if HX711_EXPERIMENTAL
	Weight = HX711_Experimental_Process(Weight);
#endif
	return Weight;
}

int32_t Get_number()
{
		  CLK_0;
		  uint32_t start_tick = HAL_GetTick();
		  uint32_t guard = HX711_READY_TIMEOUT_LOOPS;
		  while(!Read_PIN)
		  {
		  	if (((HAL_GetTick() - start_tick) > HX711_READY_TIMEOUT_MS) || (guard-- == 0U))
		  	{
		  		if (has_valid_sample)
		  		{
		  			return last_valid; /* After first good sample, reuse last on timeout */
		  		}
		  		/* Before first valid sample, restart wait to avoid seeding baseline with zero */
		  		start_tick = HAL_GetTick();
		  		guard = HX711_READY_TIMEOUT_LOOPS;
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
		  has_valid_sample = 1U;
		  last_valid = val;
		  HAL_Delay_us(1);
		  CLK_0;
		  return val;
		  
}

/* ===================== 实验性功能实现区域 ===================== */
#if HX711_EXPERIMENTAL
static long HX711_Experimental_Process(long sample)
{
	/* 仅初始化一次 */
	if (g_exp_state.start_tick == 0U)
	{
		g_exp_state.start_tick = HAL_GetTick();
		g_exp_state.minv = sample;
		g_exp_state.maxv = sample;
	}

	/* 更新滑动窗口 */
	if (g_exp_state.count < HX711_EXP_WINDOW_SIZE)
	{
		g_exp_state.buf[g_exp_state.idx] = sample;
		g_exp_state.sum += sample;
		g_exp_state.count++;
	}
	else
	{
		g_exp_state.sum -= g_exp_state.buf[g_exp_state.idx];
		g_exp_state.buf[g_exp_state.idx] = sample;
		g_exp_state.sum += sample;
	}
	g_exp_state.idx = (uint8_t)((g_exp_state.idx + 1U) % HX711_EXP_WINDOW_SIZE);

	/* 快速维护窗口内最小/最大值 */
	if (g_exp_state.count == 1U)
	{
		g_exp_state.minv = sample;
		g_exp_state.maxv = sample;
	}
	else
	{
		if (sample < g_exp_state.minv) g_exp_state.minv = sample;
		if (sample > g_exp_state.maxv) g_exp_state.maxv = sample;
	}

	/* 稳定判定：窗口满且峰峰值在阈值内 */
	const long span = g_exp_state.maxv - g_exp_state.minv;
	const uint8_t stable = (g_exp_state.count >= HX711_EXP_WINDOW_SIZE) && (span <= HX711_EXP_STABLE_SPAN);

	uint32_t now = HAL_GetTick();
	if (stable)
	{
		if (g_exp_state.stable_since == 0U)
		{
			g_exp_state.stable_since = now;
		}
	}
	else
	{
		g_exp_state.stable_since = 0U;
	}

#if HX711_EXP_ENABLE_STARTUP_TARE
	if (!g_exp_state.startup_tared)
	{
		if ((now - g_exp_state.start_tick) >= HX711_EXP_STARTUP_SETTLE_MS && stable)
		{
			g_exp_state.zero_offset = g_exp_state.sum / (long)g_exp_state.count;
			g_exp_state.startup_tared = 1U;
			g_exp_state.drift_accum = 0;
			g_exp_events |= HX711_EXP_EVT_STARTUP_TARE;
		}
	}
#endif

	long corrected = sample - g_exp_state.zero_offset;

#if HX711_EXP_ENABLE_ZERO_TRACK
	const uint8_t stable_long_enough = stable && (g_exp_state.stable_since != 0U) && ((now - g_exp_state.stable_since) >= HX711_EXP_STABLE_TIME_MS);
	if (stable_long_enough)
	{
#if HX711_EXP_ENABLE_AUTO_ZERO
		if ((corrected <= HX711_EXP_AUTOZERO_WINDOW) && (corrected >= -HX711_EXP_AUTOZERO_WINDOW))
		{
			/* 近零时，小步长拉回零点，累积漂移受限 */
			long step = (corrected > 0) ? HX711_EXP_AUTOZERO_STEP : ((corrected < 0) ? -HX711_EXP_AUTOZERO_STEP : 0);
			long new_offset = g_exp_state.zero_offset + step;
			long new_drift = g_exp_state.drift_accum + step;
			if ((new_drift <= HX711_EXP_ZERO_DRIFT_LIMIT) && (new_drift >= -HX711_EXP_ZERO_DRIFT_LIMIT))
			{
				g_exp_state.zero_offset = new_offset;
				g_exp_state.drift_accum = new_drift;
				corrected = sample - g_exp_state.zero_offset;
				g_exp_events |= HX711_EXP_EVT_AUTOZERO;
			}
		}
#endif
	}
#endif

	return corrected;
}
#endif /* HX711_EXPERIMENTAL */
/* =================== END EXPERIMENTAL FEATURES (implementation) ==================== */

/* 实验性事件获取：返回并清除事件标志 */
uint32_t HX711_GetAndClearExpEvents(void)
{
#if HX711_EXPERIMENTAL
	uint32_t ev = g_exp_events;
	g_exp_events = 0U;
	return ev;
#else
	return 0U;
#endif
}