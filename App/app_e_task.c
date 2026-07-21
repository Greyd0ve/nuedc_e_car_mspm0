#include "app_e_task.h"
#include "app_config.h"
#include "app_e_car.h"
#include "app_visual_gimbal_xy.h"
#include "BeepLed.h"
#include "Key.h"

static ETaskState_t s_state;
static uint16_t s_aimStableMs;

void ETask_Init(void)
{
    s_state        = E_TASK_IDLE;
    s_aimStableMs  = 0U;
}

void ETask_Start(void)
{
    s_aimStableMs = 0U;
    s_state       = E_TASK_WAIT_AIM;
}

void ETask_Stop(void)
{
    ECar_Stop();
    s_aimStableMs = 0U;
    s_state       = E_TASK_IDLE;
}

void ETask_Reset(void)
{
    ECar_Reset();
    s_aimStableMs = 0U;
    s_state       = E_TASK_IDLE;
}

void ETask_Control10ms(void)
{
    if (s_state == E_TASK_WAIT_AIM)
    {
        if (VisualGimbalXY_IsLocked())
        {
            if (s_aimStableMs < ECAR_E_TASK_AIM_STABLE_MS)
            {
                s_aimStableMs += ECAR_CONTROL_PERIOD_MS;
            }

            if (s_aimStableMs >= ECAR_E_TASK_AIM_STABLE_MS)
            {
                ECar_Start();
                s_state = E_TASK_RUNNING;
            }
        }
        else
        {
            s_aimStableMs = 0U;
        }
    }
    else if (s_state == E_TASK_RUNNING)
    {
        if (ECar_GetState() == E_CAR_FINISH)
        {
            s_state = E_TASK_FINISH;
        }
    }
}

void ETask_KeyProcess(void)
{
    uint8_t key = Key_GetNum();
    if (key == 0U) return;

    if (key == 1U)
    {
        if (s_state == E_TASK_IDLE || s_state == E_TASK_FINISH)
        {
            uint8_t lap = ECar_GetTargetLap();
            lap++;
            if (lap > 5U) { lap = 1U; }
            ECar_SetTargetLap(lap);
            LED_User_BlinkTimes(lap, 100U);
        }
        return;
    }

    if (key == 2U)
    {
        if (s_state == E_TASK_IDLE || s_state == E_TASK_FINISH)
        {
            ETask_Start();
        }
        return;
    }

    if (key == 3U)
    {
        ETask_Stop();
        return;
    }

    if (key == 4U)
    {
        ETask_Reset();
    }
}

ETaskState_t ETask_GetState(void)     { return s_state; }
uint16_t    ETask_GetAimStableMs(void) { return s_aimStableMs; }
