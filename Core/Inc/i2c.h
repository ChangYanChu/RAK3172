/*
 * i2c.h
 *
 * Generated manually to provide I2C1 support for SC7A20 sensor.
 */

#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void);
void I2C2_Scan(void);
HAL_StatusTypeDef I2C2_BusRecover(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
