/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#ifndef HX711_EXPERIMENTAL
#define HX711_EXPERIMENTAL 1 /* 总开关：1 开启实验性功能，0 关闭 */
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "hx711.h"
#include "OLED.h"
#include "Filteringalgorithm.h"
#include "Linearization.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

long weight = 0;
static long raw_display = 0;
float weight_value = 0.0f;
static float display_value = 0.0f;
static int32_t locked_int = 0;
static uint8_t display_locked = 0U;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xff);
  return (ch);
}

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t DecimalLength);
static void OLED_ShowHeader(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void OLED_ShowHeader(void)
{
  OLED_ShowString(1, 1, "RawCnt:");
  OLED_ShowString(3, 1, "Weight(g):");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_ShowHeader();
  OLED_ShowString(4, 1, "Wait HX711...  ");
  (void)Get_number();

  OLED_ShowString(4, 1, "Stabilizing    ");
  HAL_Delay(5000);
  OLED_ShowString(4, 1, "                ");
  HAL_Delay(1000);
  /* 过滤初始化在 Get_Weight 内延迟进行，此处无需去皮基线 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    weight = Get_Weight();
    raw_display = weight;
    weight_value = Linearization_Apply((float)weight);
    if ((weight_value < 0.5f) && (weight_value > -0.5f))
    {
      weight_value = 0.0f;
    }

    int32_t current_int = (int32_t)(weight_value >= 0.0f ? (weight_value + 0.5f) : (weight_value - 0.5f));

    if (display_locked)
    {
      if (current_int != locked_int)
      {
        display_locked = 0U;
      }
    }

    if (!display_locked)
    {
      static int32_t last_int = 0;
      static uint32_t last_change_tick = 0U;

      if (current_int != last_int)
      {
        last_int = current_int;
        last_change_tick = HAL_GetTick();
      }
      else if ((current_int != 0) && ((HAL_GetTick() - last_change_tick) >= 3000U))
      {
        display_locked = 1U;
        locked_int = current_int;
        display_value = weight_value;
      }

      if (!display_locked)
      {
        display_value = weight_value;
      }
    }

#if HX711_EXPERIMENTAL
    uint32_t exp_ev = HX711_GetAndClearExpEvents();
    if (exp_ev != 0U)
    {
      OLED_ShowString(1, 1, " EXPERIMENTAL   ");
      OLED_ShowString(2, 1, " EVENT TRIGGER  ");
      if (exp_ev & HX711_EXP_EVT_STARTUP_TARE)
      {
        OLED_ShowString(3, 1, " Startup Tare   ");
      }
      else
      {
        OLED_ShowString(3, 1, "                ");
      }
      if (exp_ev & HX711_EXP_EVT_AUTOZERO)
      {
        OLED_ShowString(4, 1, " Auto Zero Adj  ");
      }
      else
      {
        OLED_ShowString(4, 1, "                ");
      }
      HAL_Delay(1000);
      OLED_ShowHeader();
    }
#endif

    OLED_ShowFloat(2, 1, (float)raw_display, 0);
    OLED_ShowFloat(4, 1, display_value, 2);
    printf("%.2f\r\n", (double)display_value);
    HAL_Delay(120);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
