#include "Encoder.h"
#include "Board_Config.h"
#include "cmsis_compiler.h"

static volatile int32_t s_leftDelta = 0;
static volatile int32_t s_rightDelta = 0;
static volatile uint8_t s_leftALast = 0U;
static volatile uint8_t s_rightALast = 0U;
static volatile uint32_t s_leftIsrCount = 0U;
static volatile uint32_t s_leftSameAIgnored = 0U;
static volatile uint32_t s_leftStatusCount = 0U;
static volatile uint32_t s_rightIsrCount = 0U;
static volatile uint32_t s_rightSameAIgnored = 0U;
static volatile uint32_t s_rightStatusCount = 0U;

#ifndef ENCODER_COUNT_A_RISE_ONLY
#define ENCODER_COUNT_A_RISE_ONLY 1U
#endif

static uint8_t Encoder_ReadLevel(GPIO_Regs *port, uint32_t pin)
{
    return (DL_GPIO_readPins(port, pin) != 0U) ? 1U : 0U;
}

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
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_leftALast = Encoder_ReadLevel(ENC_L_A_PORT, ENC_L_A_PIN);
    s_rightALast = Encoder_ReadLevel(ENC_R_A_PORT, ENC_R_A_PIN);
    s_leftDelta = 0;
    s_rightDelta = 0;
    s_leftIsrCount = 0U;
    s_leftSameAIgnored = 0U;
    s_leftStatusCount = 0U;
    s_rightIsrCount = 0U;
    s_rightSameAIgnored = 0U;
    s_rightStatusCount = 0U;

    DL_GPIO_clearInterruptStatus(ENC_L_A_PORT, ENC_L_A_PIN);
    DL_GPIO_clearInterruptStatus(ENC_R_A_PORT, ENC_R_A_PIN);
    NVIC_ClearPendingIRQ(ENCODER_GPIO_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIO_IRQN);

    if (primask == 0U)
    {
        __enable_irq();
    }
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

uint32_t Encoder_GetRightIsrCount(void)
{
    return s_rightIsrCount;
}

uint32_t Encoder_GetRightSameAIgnored(void)
{
    return s_rightSameAIgnored;
}

uint32_t Encoder_GetRightStatusCount(void)
{
    return s_rightStatusCount;
}

static void Encoder_HandleLeftA(void)
{
    uint8_t a = Encoder_ReadLevel(ENC_L_A_PORT, ENC_L_A_PIN);
    uint8_t b;
    int32_t dir;

    s_leftIsrCount++;
    if (a == s_leftALast)
    {
        s_leftSameAIgnored++;
        return;
    }
    s_leftALast = a;

#if ENCODER_COUNT_A_RISE_ONLY
    if (a == 0U)
    {
        return;
    }
#endif

    b = Encoder_ReadLevel(ENC_L_B_PORT, ENC_L_B_PIN);
    dir = (a == b) ? 1 : -1;
    s_leftDelta += dir * LEFT_ENCODER_DIR;
}

static void Encoder_HandleRightA(void)
{
    uint8_t a = Encoder_ReadLevel(ENC_R_A_PORT, ENC_R_A_PIN);
    uint8_t b;
    int32_t dir;

    s_rightIsrCount++;
    if (a == s_rightALast)
    {
        s_rightSameAIgnored++;
        return;
    }
    s_rightALast = a;

#if ENCODER_COUNT_A_RISE_ONLY
    if (a == 0U)
    {
        return;
    }
#endif

    b = Encoder_ReadLevel(ENC_R_B_PORT, ENC_R_B_PIN);
    dir = (a == b) ? 1 : -1;
    s_rightDelta += dir * RIGHT_ENCODER_DIR;
}

static void Encoder_ServicePort(GPIO_Regs *port)
{
    uint32_t mask = 0U;
    uint32_t status;

    if (ENC_L_A_PORT == port)
    {
        mask |= ENC_L_A_PIN;
    }
    if (ENC_R_A_PORT == port)
    {
        mask |= ENC_R_A_PIN;
    }
    if (mask == 0U)
    {
        return;
    }

    status = DL_GPIO_getEnabledInterruptStatus(port, mask);
    DL_GPIO_clearInterruptStatus(port, status);

    if ((ENC_L_A_PORT == port) && ((status & ENC_L_A_PIN) != 0U))
    {
        s_leftStatusCount++;
        Encoder_HandleLeftA();
    }
    if ((ENC_R_A_PORT == port) && ((status & ENC_R_A_PIN) != 0U))
    {
        s_rightStatusCount++;
        Encoder_HandleRightA();
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            Encoder_ServicePort(GPIOB);
            break;

        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            Encoder_ServicePort(GPIOA);
            break;

        default:
            break;
    }
}
