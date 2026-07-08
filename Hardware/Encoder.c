#include "Encoder.h"
#include "Board_Config.h"
#include "cmsis_compiler.h"

static volatile int32_t s_leftDelta = 0;
static volatile int32_t s_rightDelta = 0;

static int16_t Encoder_LimitDelta(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

void Encoder_Init(void)
{
    Encoder_ClearAll();

    DL_GPIO_clearInterruptStatus(ENC_L_A_PORT, ENC_L_A_PIN);
    DL_GPIO_clearInterruptStatus(ENC_R_A_PORT, ENC_R_A_PIN);
    NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIO_IRQN);
}

int16_t Encoder_GetLeftDelta(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = s_leftDelta;
    s_leftDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return Encoder_LimitDelta(value);
}

int16_t Encoder_GetRightDelta(void)
{
    int32_t value;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    value = s_rightDelta;
    s_rightDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return Encoder_LimitDelta(value);
}

void Encoder_ClearAll(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_leftDelta = 0;
    s_rightDelta = 0;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void Encoder_HandleLeftA(void)
{
    int32_t dir = (DL_GPIO_readPins(ENC_L_B_PORT, ENC_L_B_PIN) != 0U) ? 1 : -1;
    s_leftDelta += dir * LEFT_ENCODER_DIR;
}

static void Encoder_HandleRightA(void)
{
    int32_t dir = (DL_GPIO_readPins(ENC_R_B_PORT, ENC_R_B_PIN) != 0U) ? 1 : -1;
    s_rightDelta += dir * RIGHT_ENCODER_DIR;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
        {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENC_L_A_PORT, ENC_L_A_PIN);
            DL_GPIO_clearInterruptStatus(ENC_L_A_PORT, status);
            if ((status & ENC_L_A_PIN) != 0U)
            {
                Encoder_HandleLeftA();
            }
            break;
        }

        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
        {
            uint32_t status = DL_GPIO_getEnabledInterruptStatus(ENC_R_A_PORT, ENC_R_A_PIN);
            DL_GPIO_clearInterruptStatus(ENC_R_A_PORT, status);
            if ((status & ENC_R_A_PIN) != 0U)
            {
                Encoder_HandleRightA();
            }
            break;
        }

        default:
            break;
    }
}
