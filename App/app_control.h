#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include <stdint.h>

void App_Control_Init(void);
void App_Control_UpdatePIDParam(void);
void App_Control_ResetPID(void);
void App_Control_ForcePWMZero(void);
/* Updates encoder totals in pulses and wheel speeds in cm/s over periodMs. */
void App_Control_UpdateEncoderSpeed(uint16_t periodMs);
void App_Control_ApplyMotorOutput(void);

#endif
