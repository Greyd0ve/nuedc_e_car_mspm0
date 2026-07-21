#ifndef __APP_E_TASK_H
#define __APP_E_TASK_H

#include <stdint.h>

typedef enum
{
    E_TASK_IDLE = 0,
    E_TASK_WAIT_AIM,
    E_TASK_RUNNING,
    E_TASK_FINISH
} ETaskState_t;

void ETask_Init(void);
void ETask_Start(void);
void ETask_Stop(void);
void ETask_Reset(void);
void ETask_Control10ms(void);
void ETask_KeyProcess(void);

ETaskState_t ETask_GetState(void);
uint16_t ETask_GetAimStableMs(void);

#endif
