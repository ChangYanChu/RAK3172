/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "app_lorawan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "i2c.h"
#include "sc7A20_driver.h"
#include "sys_app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t sc7a20_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 移除旧的 I2C1 扫描函数，改为使用 i2c.c 中的 I2C2_Scan */

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
  MX_LoRaWAN_Init();
  /* USER CODE BEGIN 2 */
  MX_I2C2_Init();
  HAL_Delay(10); /* 给予总线与传感器上电稳定时间 */
  uint8_t sc7a20_id = 0U;

  HAL_StatusTypeDef init_status = SC7A20_Init();
  /* 设置为 4g 量程提高动态范围并便于测试 */
  if (init_status == HAL_OK)
  {
    SC7A20_SetRange(SC7A20_RANGE_4G);
  }

  if (init_status == HAL_OK)
  {
    sc7a20_ready = 1U;
    if (SC7A20_ReadID(&sc7a20_id) == HAL_OK)
    {
      APP_LOG(TS_OFF, VLEVEL_L, "SC7A20 WHO_AM_I: 0x%02X (addr=0x%02X)\r\n", sc7a20_id, SC7A20_GetActiveAddress());
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_L, "SC7A20 WHO_AM_I read failed\r\n");
    }
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_L, "SC7A20 init failed (status=%d), start diagnostic scan\r\n", init_status);
    I2C2_Scan();
    APP_LOG(TS_OFF, VLEVEL_L, "Attempt bus recover...\r\n");
    if (I2C2_BusRecover() == HAL_OK)
    {
      APP_LOG(TS_OFF, VLEVEL_L, "Bus recover done, re-init I2C2 & sensor\r\n");
      MX_I2C2_Init();
      HAL_Delay(10);
      init_status = SC7A20_Init();
      if (init_status == HAL_OK)
      {
        sc7a20_ready = 1U;
        if (SC7A20_ReadID(&sc7a20_id) == HAL_OK)
        {
          APP_LOG(TS_OFF, VLEVEL_L, "SC7A20 WHO_AM_I (retry): 0x%02X (addr=0x%02X)\r\n", sc7a20_id, SC7A20_GetActiveAddress());
        }
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_L, "SC7A20 init retry failed (status=%d)\r\n", init_status);
        I2C2_Scan();
      }
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_L, "Bus recover failed, skip sensor init retry\r\n");
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 简单滑动平均缓冲 */
  #define AVG_DEPTH 16
  int32_t avg_x = 0, avg_y = 0, avg_z = 0; /* 累加值（12bit 对齐数据） */
  int16_t ring_x[AVG_DEPTH] = {0};
  int16_t ring_y[AVG_DEPTH] = {0};
  int16_t ring_z[AVG_DEPTH] = {0};
  uint8_t ring_idx = 0;
  uint8_t filled = 0;
  uint32_t last_print = 0;

  while (1)
  {
    MX_LoRaWAN_Process();

    if (sc7a20_ready)
    {
      /* 读取一次 12bit 对齐数据 */
      int16_t x12, y12, z12;
  if (SC7A20_ReadAccel(&x12, &y12, &z12) == HAL_OK)
      {
        /* 从累加中移除旧值 */
        avg_x -= ring_x[ring_idx];
        avg_y -= ring_y[ring_idx];
        avg_z -= ring_z[ring_idx];
        /* 写入新值 */
        ring_x[ring_idx] = x12;
        ring_y[ring_idx] = y12;
        ring_z[ring_idx] = z12;
        avg_x += x12;
        avg_y += y12;
        avg_z += z12;
        ring_idx = (ring_idx + 1) % AVG_DEPTH;
        if (filled < AVG_DEPTH) { filled++; }
      }

      uint32_t now = HAL_GetTick();
      if ((now - last_print) >= 1000U && filled > 0)
      {
        float mg_per_lsb = SC7A20_GetMgPerLsb();
        /* 输出整数 mg（乘法后四舍五入） */
        int32_t ix = (int32_t)((avg_x * mg_per_lsb) / filled + 0.5f);
        int32_t iy = (int32_t)((avg_y * mg_per_lsb) / filled + 0.5f);
        int32_t iz = (int32_t)((avg_z * mg_per_lsb) / filled + 0.5f);
  char line[128];
  /* 手动拼接：深度 / mg 数值 / 量程字符串 / 地址 */
  const char* rangeStr = (SC7A20_GetRange()==SC7A20_RANGE_2G)?"2g":(SC7A20_GetRange()==SC7A20_RANGE_4G?"4g":"8g");
  int addr = SC7A20_GetActiveAddress();
  /* 简单整数转字符串 */
  char buf_ix[12]; char buf_iy[12]; char buf_iz[12];
  snprintf(buf_ix, sizeof(buf_ix), "%ld", (long)ix);
  snprintf(buf_iy, sizeof(buf_iy), "%ld", (long)iy);
  snprintf(buf_iz, sizeof(buf_iz), "%ld", (long)iz);
  snprintf(line, sizeof(line), "SC7A20 AVG depth=%u mg X=%s Y=%s Z=%s range=%s addr=0x%02X\r\n",
     filled, buf_ix, buf_iy, buf_iz, rangeStr, addr);
  APP_LOG(TS_OFF, VLEVEL_M, "%s", line);
        last_print = now;
      }
    }
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

  /** Configure LSE Drive Capability
   */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
