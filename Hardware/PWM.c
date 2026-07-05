#include "PWM.h"
#include "Board_Config.h"

static uint16_t PWM_Clamp(uint16_t compare)
{
    return (compare > PWM_MAX_DUTY) ? PWM_MAX_DUTY : compare;
}

static uint32_t PWM_ToTimerCompare(uint16_t duty)
{
    uint32_t dutyCounts;

    duty = PWM_Clamp(duty);
    dutyCounts = ((uint32_t)MOTOR_PWM_PERIOD_COUNTS * (uint32_t)duty) / PWM_MAX_DUTY;

    if (dutyCounts == 0U)
    {
        return MOTOR_PWM_PERIOD_COUNTS;
    }
    if (dutyCounts >= MOTOR_PWM_PERIOD_COUNTS)
    {
        return 0U;
    }
    return MOTOR_PWM_PERIOD_COUNTS - dutyCounts;
}

void PWM_Init(void)
{
    PWM_SetCompareA(0U);
    PWM_SetCompareB(0U);
    DL_TimerG_startCounter(MOTOR_PWM_TIMER_INST);
}

void PWM_SetCompareA(uint16_t compare)
{
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_TIMER_INST, PWM_ToTimerCompare(compare), MOTOR_PWM_LEFT_CC_INDEX);
}

void PWM_SetCompareB(uint16_t compare)
{
    DL_TimerG_setCaptureCompareValue(
        MOTOR_PWM_TIMER_INST, PWM_ToTimerCompare(compare), MOTOR_PWM_RIGHT_CC_INDEX);
}
