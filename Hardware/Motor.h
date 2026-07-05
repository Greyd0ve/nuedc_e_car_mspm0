#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_SetLeftPWM(int16_t pwm);
void Motor_SetRightPWM(int16_t pwm);
void Motor_SetPWM(int16_t leftPwm, int16_t rightPwm);
void Motor_StopAll(void);

#endif
