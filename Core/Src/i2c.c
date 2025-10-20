/*
 * i2c.c
 *
 * I2C2 初始化及调试工具 (PA11 SDA, PA12 SCL) 用于 SC7A20 / 其它传感器。
 */

#include "main.h"
#include "i2c.h"
#include "sys_app.h"
#include "stm32wlxx_hal_i2c.h"
#include "stm32wlxx_hal_gpio.h"

I2C_HandleTypeDef hi2c2;

static void I2C2_PrintRegs(void)
{
  APP_LOG(TS_OFF, VLEVEL_L, "I2C2 REG TIMINGR=0x%08lX CR1=0x%08lX CR2=0x%08lX ISR=0x%08lX\r\n",
          I2C2->TIMINGR, I2C2->CR1, I2C2->CR2, I2C2->ISR);
}

HAL_StatusTypeDef I2C2_BusRecover(void)
{
  /* 检测 SDA 持续低电平 (总线卡死) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  uint32_t sda_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  uint32_t scl_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  if (sda_state == GPIO_PIN_SET && scl_state == GPIO_PIN_SET)
  {
    return HAL_OK; /* 总线空闲，无需恢复 */
  }

  APP_LOG(TS_OFF, VLEVEL_L, "I2C2 bus recovery start (SDA=%lu SCL=%lu)\r\n", sda_state, scl_state);

  /* 如果 I2C2 已经初始化则暂时关闭外设避免与手动 GPIO 冲突 */
  if ((I2C2->CR1 & I2C_CR1_PE) != 0U)
  {
    CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
  }

  /* 改为普通输出模式产生 9 个 SCL 脉冲释放被卡住的从设备 */
  GPIO_InitStruct.Pin = GPIO_PIN_12; /* SCL */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11; /* SDA 保持输入观察 */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(1);
  }
  /* 生成一个 STOP 条件: SCL 高, SDA 拉高 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(1);

  /* 重新配置为 I2C AF */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL; /* 外部上拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 重新使能 I2C 外设 (不重新完整初始化，保持之前的寄存器配置) */
  SET_BIT(I2C2->CR1, I2C_CR1_PE);

  APP_LOG(TS_OFF, VLEVEL_L, "I2C2 bus recovery done\r\n");
  return HAL_OK;
}

void MX_I2C2_Init(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D; /* 由 Cube 生成的稳定 100kHz 配置 */
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) { Error_Handler(); }

  I2C2_PrintRegs();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12; /* PA11 SDA, PA12 SCL */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL; /* 使用外部上拉 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  }
}

void I2C2_Scan(void)
{
  APP_LOG(TS_OFF, VLEVEL_L, "I2C2 scan start\r\n");
  I2C2_PrintRegs();
  HAL_Delay(50);

  if (I2C2_BusRecover() != HAL_OK)
  {
    APP_LOG(TS_OFF, VLEVEL_L, "Bus recover failed\r\n");
  }

  uint8_t found = 0U;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    for (int attempt = 0; attempt < 3; attempt++)
    {
      if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 1, 50) == HAL_OK)
      {
        APP_LOG(TS_OFF, VLEVEL_L, "Found device @0x%02X\r\n", addr);
        found = 1U;
        break;
      }
    }
  }

  if (!found)
  {
    APP_LOG(TS_OFF, VLEVEL_L, "I2C2 scan end: no device\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_L, "I2C2 scan end\r\n");
  }
}
