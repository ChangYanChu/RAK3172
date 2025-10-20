#ifndef SC7A20_DRIVER_H
#define SC7A20_DRIVER_H

#include <stdint.h>
#include "stm32wlxx_hal.h"

HAL_StatusTypeDef SC7A20_Init(void);
HAL_StatusTypeDef SC7A20_ReadID(uint8_t *id);
HAL_StatusTypeDef SC7A20_ReadAccel(int16_t *x, int16_t *y, int16_t *z);
/* 读取未右移处理的 16 位原始数据（保留低 4 位） */
HAL_StatusTypeDef SC7A20_ReadAccelRaw(int16_t *x, int16_t *y, int16_t *z);
uint8_t SC7A20_GetActiveAddress(void);

/* 可选: 设置量程(2/4/8g)，返回 HAL_OK 成功 */
typedef enum {
	SC7A20_RANGE_2G = 0,
	SC7A20_RANGE_4G,
	SC7A20_RANGE_8G
} SC7A20_Range_t;
HAL_StatusTypeDef SC7A20_SetRange(SC7A20_Range_t range);
float SC7A20_GetMgPerLsb(void); /* 根据当前量程返回每 LSB 对应 mg */
SC7A20_Range_t SC7A20_GetRange(void);


#endif // SC7A20_DRIVER_H
