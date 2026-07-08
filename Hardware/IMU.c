#include "IMU.h"
#include "Board_Config.h"

#define MPU6050_ADDR          0x68U
#define MPU6050_REG_WHO_AM_I  0x75U
#define MPU6050_I2C_TIMEOUT   100000U

static uint8_t s_imuReady = 0U;

static uint8_t IMU_WaitIdle(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
        {
            return 0U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U)
        {
            return 1U;
        }
        timeout--;
    }

    return 0U;
}

static uint8_t IMU_WaitDone(void)
{
    uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(MPU6050_I2C_INST);
        if ((status & DL_I2C_CONTROLLER_STATUS_BUSY) == 0U)
        {
            return (uint8_t)(((status & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) ? 1U : 0U);
        }
        timeout--;
    }

    return 0U;
}

void IMU_Init(void)
{
    uint8_t who = 0U;
    s_imuReady = IMU_ReadWhoAmI(&who);
}

uint8_t IMU_ReadWhoAmI(uint8_t *whoAmI)
{
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    uint16_t written;

    if (whoAmI == 0)
    {
        return 0U;
    }

    *whoAmI = 0U;
    if (!IMU_WaitIdle())
    {
        DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    }

    DL_I2C_resetControllerTransfer(MPU6050_I2C_INST);
    written = DL_I2C_fillControllerTXFIFO(MPU6050_I2C_INST, &reg, 1U);
    if (written != 1U)
    {
        return 0U;
    }
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1U);
    if (!IMU_WaitDone())
    {
        return 0U;
    }

    DL_I2C_startFlushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_stopFlushControllerRXFIFO(MPU6050_I2C_INST);
    DL_I2C_startControllerTransfer(MPU6050_I2C_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1U);
    if (!IMU_WaitDone())
    {
        return 0U;
    }

    if (DL_I2C_isControllerRXFIFOEmpty(MPU6050_I2C_INST))
    {
        return 0U;
    }

    *whoAmI = DL_I2C_receiveControllerData(MPU6050_I2C_INST);
    return 1U;
}

uint8_t IMU_IsReady(void)
{
    return s_imuReady;
}
