#include "Motor.h"
#include "Board_Config.h"
#include "PWM.h"

static int16_t Motor_ClampPWM(int16_t pwm)
{
    if (pwm > (int16_t)PWM_MAX_DUTY)
    {
        return (int16_t)PWM_MAX_DUTY;
    }
    if (pwm < -(int16_t)PWM_MAX_DUTY)
    {
        return -(int16_t)PWM_MAX_DUTY;
    }
    return pwm;
}

static void Motor_SetDirection(
    GPIO_Regs *in1Port, uint32_t in1Pin, GPIO_Regs *in2Port, uint32_t in2Pin, int16_t pwm)
{
    if (pwm > 0)
    {
        DL_GPIO_setPins(in1Port, in1Pin);
        DL_GPIO_clearPins(in2Port, in2Pin);
    }
    else if (pwm < 0)
    {
        DL_GPIO_clearPins(in1Port, in1Pin);
        DL_GPIO_setPins(in2Port, in2Pin);
    }
    else
    {
        DL_GPIO_clearPins(in1Port, in1Pin);
        DL_GPIO_clearPins(in2Port, in2Pin);
    }
}

void Motor_Init(void)
{
#if MOTOR_USE_STBY
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
#endif
    PWM_Init();
    Motor_StopAll();
#if MOTOR_USE_STBY
    DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_PIN);
#endif
}

void Motor_SetLeftPWM(int16_t pwm)
{
    int16_t signedPwm = Motor_ClampPWM((int16_t)(pwm * LEFT_MOTOR_DIR_SIGN));
    uint16_t duty = (signedPwm < 0) ? (uint16_t)(-signedPwm) : (uint16_t)signedPwm;

    Motor_SetDirection(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, signedPwm);
    PWM_SetCompareA(duty);
}

void Motor_SetRightPWM(int16_t pwm)
{
    int16_t signedPwm = Motor_ClampPWM((int16_t)(pwm * RIGHT_MOTOR_DIR_SIGN));
    uint16_t duty = (signedPwm < 0) ? (uint16_t)(-signedPwm) : (uint16_t)signedPwm;

    Motor_SetDirection(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, signedPwm);
    PWM_SetCompareB(duty);
}

void Motor_SetPWM(int16_t leftPwm, int16_t rightPwm)
{
    Motor_SetLeftPWM(leftPwm);
    Motor_SetRightPWM(rightPwm);
}

void Motor_StopAll(void)
{
    PWM_SetCompareA(0U);
    PWM_SetCompareB(0U);
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
}
