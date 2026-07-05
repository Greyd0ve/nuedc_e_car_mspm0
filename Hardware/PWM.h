#ifndef __PWM_H
#define __PWM_H

#include <stdint.h>

#define PWM_MAX_DUTY 1000U

void PWM_Init(void);
void PWM_SetCompareA(uint16_t compare);
void PWM_SetCompareB(uint16_t compare);

#endif
