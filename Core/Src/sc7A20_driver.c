#include "sc7A20_driver.h"
#include "i2c.h"

#define SC7A20_I2C_ADDRESS_PRIMARY   0x18
#define SC7A20_I2C_ADDRESS_SECONDARY 0x19U /* 若 SA0 上拉/下拉差异或兼容封装可出现该地址 */
static uint8_t sc7a20_active_address = SC7A20_I2C_ADDRESS_PRIMARY;

#define SC7A20_REG_WHO_AM_I      0x0FU
#define SC7A20_REG_CTRL_REG1     0x20U
#define SC7A20_REG_CTRL_REG4     0x23U
#define SC7A20_REG_OUT_X_L       0x28U
#define SC7A20_REG_OUT_Y_L       0x2AU
#define SC7A20_REG_OUT_Z_L       0x2CU

#define SC7A20_AUTO_INCREMENT    0x80U

#define SC7A20_CTRL1_DEFAULT     0x47U
/* CTRL_REG4 位设置: BDU=1(0x80), 高分辨率=0x08, SPI 模式=0, 量程 FS bits(4g=01,8g=10,2g=00) */
#define SC7A20_CTRL4_DEFAULT     0x88U /* BDU=1 + HR=1 + FS=00 => 2g 高分辨率 */
static SC7A20_Range_t sc7a20_range = SC7A20_RANGE_2G;

#define SC7A20_I2C_TIMEOUT_MS    500U

static HAL_StatusTypeDef SC7A20_WriteRegister(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef SC7A20_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length);
uint8_t SC7A20_GetActiveAddress(void) { return sc7a20_active_address; }


// 中文注释
// 用于初始化传感器
HAL_StatusTypeDef SC7A20_Init(void)
{
    /* 先尝试主地址 */
    sc7a20_active_address = SC7A20_I2C_ADDRESS_PRIMARY;
    HAL_StatusTypeDef status = SC7A20_WriteRegister(SC7A20_REG_CTRL_REG1, SC7A20_CTRL1_DEFAULT);
    if (status != HAL_OK)
    {
        /* 切换次地址再试一次 */
        sc7a20_active_address = SC7A20_I2C_ADDRESS_SECONDARY;
        status = SC7A20_WriteRegister(SC7A20_REG_CTRL_REG1, SC7A20_CTRL1_DEFAULT);
        if (status != HAL_OK)
        {
            return status;
        }
    }
    /* CTRL_REG4 */
        HAL_StatusTypeDef s2 = SC7A20_WriteRegister(SC7A20_REG_CTRL_REG4, SC7A20_CTRL4_DEFAULT);
        if (s2 == HAL_OK)
        {
            sc7a20_range = SC7A20_RANGE_2G;
        }
        return s2;
}

// 中文注释
// 用于读写设备 ID 的辅助函数   
HAL_StatusTypeDef SC7A20_ReadID(uint8_t *id)
{
    if (id == NULL)
    {
        return HAL_ERROR;
    }

    return SC7A20_ReadRegisters(SC7A20_REG_WHO_AM_I, id, 1U);
}

// 中文注释
// 用于读写寄存器的辅助函数
HAL_StatusTypeDef SC7A20_ReadAccel(int16_t *x, int16_t *y, int16_t *z)
{

    if ((x == NULL) || (y == NULL) || (z == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t raw[6] = {0};
    HAL_StatusTypeDef status = SC7A20_ReadRegisters(SC7A20_REG_OUT_X_L | SC7A20_AUTO_INCREMENT, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    *x = (int16_t)((raw[1] << 8) | raw[0]) >> 4;
    *y = (int16_t)((raw[3] << 8) | raw[2]) >> 4;
    *z = (int16_t)((raw[5] << 8) | raw[4]) >> 4;

    return HAL_OK;
}

/* 读取原始未右移的 16 位数值，调用者可自行根据量程缩放或右移 4 位 */
HAL_StatusTypeDef SC7A20_ReadAccelRaw(int16_t *x, int16_t *y, int16_t *z)
{
    if ((x == NULL) || (y == NULL) || (z == NULL))
    {
        return HAL_ERROR;
    }

    uint8_t raw[6] = {0};
    HAL_StatusTypeDef status = SC7A20_ReadRegisters(SC7A20_REG_OUT_X_L | SC7A20_AUTO_INCREMENT, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    *x = (int16_t)((raw[1] << 8) | raw[0]);
    *y = (int16_t)((raw[3] << 8) | raw[2]);
    *z = (int16_t)((raw[5] << 8) | raw[4]);
    return HAL_OK;
}


// 中文注释 
// 用于读写寄存器的辅助函数
static HAL_StatusTypeDef SC7A20_WriteRegister(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c2,
                             (sc7a20_active_address << 1),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             SC7A20_I2C_TIMEOUT_MS);
}

// 读取多个寄存器的辅助函数
static HAL_StatusTypeDef SC7A20_ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    return HAL_I2C_Mem_Read(&hi2c2,
                            (sc7a20_active_address << 1),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            length,
                            SC7A20_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef SC7A20_SetRange(SC7A20_Range_t range)
{
    uint8_t ctrl4 = 0x80 /* BDU */ | 0x08 /* HR 高分辨率 */;
    switch(range)
    {
        case SC7A20_RANGE_2G: ctrl4 |= 0x00; break; /* FS=00 */
        case SC7A20_RANGE_4G: ctrl4 |= 0x10; break; /* FS=01 */
        case SC7A20_RANGE_8G: ctrl4 |= 0x20; break; /* FS=10 */
        default: return HAL_ERROR;
    }
    HAL_StatusTypeDef st = SC7A20_WriteRegister(SC7A20_REG_CTRL_REG4, ctrl4);
    if (st == HAL_OK) { sc7a20_range = range; }
    return st;
}

float SC7A20_GetMgPerLsb(void)
{
    /* 高分辨率模式下典型灵敏度：2g=0.98mg/LSB,4g=1.95mg/LSB,8g=3.9mg/LSB (示例值，需按数据手册校准) */
    switch(sc7a20_range)
    {
        case SC7A20_RANGE_2G: return 0.98f;
        case SC7A20_RANGE_4G: return 1.95f;
        case SC7A20_RANGE_8G: return 3.90f;
        default: return 1.0f;
    }
}

SC7A20_Range_t SC7A20_GetRange(void)
{
    return sc7a20_range;
}
