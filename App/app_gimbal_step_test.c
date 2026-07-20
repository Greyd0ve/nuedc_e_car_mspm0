#include "app_gimbal_step_test.h"
#include "app_config.h"
#include "GimbalStepper.h"
#include "Key.h"
#include "Serial.h"
#include "Board_Config.h"
#include <stdint.h>

static uint8_t s_initDone = 0U;
static uint8_t s_k3WasPressed = 0U;

static void App_CheckDone(void)
{
    if (GimbalStepper_IsDone(GIMBAL_AXIS_X))
    {
        GimbalStepper_ClearDone(GIMBAL_AXIS_X);
        Serial_Printf("[gimbal,x,done]\r\n");
    }

    if (GimbalStepper_IsDone(GIMBAL_AXIS_Y))
    {
        GimbalStepper_ClearDone(GIMBAL_AXIS_Y);
        Serial_Printf("[gimbal,y,done]\r\n");
    }
}

static void App_CheckK3Emergency(void)
{
    if (Key_IsPressed(3U) != 0U)
    {
        if (s_k3WasPressed == 0U)
        {
            s_k3WasPressed = 1U;
            GimbalStepper_StopAll();
            Serial_Printf("[gimbal,stop,all]\r\n");
        }
    }
    else
    {
        s_k3WasPressed = 0U;
    }
}

void GimbalStepTest_Init(void)
{
    s_initDone = 1U;
    s_k3WasPressed = 0U;
    GimbalStepper_Init();
}

void GimbalStepTest_KeyProcess(void)
{
    uint8_t key;

    if (s_initDone == 0U) { return; }

    App_CheckDone();
    App_CheckK3Emergency();

    key = Key_GetNum();
    if (key == 0U) { return; }

    if (key == 1U)
    {
        if (GimbalStepper_IsBusy(GIMBAL_AXIS_X) != 0U)
        {
            Serial_Printf("[gimbal,x,busy]\r\n");
        }
        else if (GimbalStepper_StartMove(GIMBAL_AXIS_X,
                     GIMBAL_DIR_POSITIVE,
                     GIMBAL_TEST_PULSES, GIMBAL_TEST_STEP_FREQ_HZ))
        {
            Serial_Printf("[gimbal,x,start,%lu,%lu]\r\n",
                (unsigned long)GIMBAL_TEST_PULSES,
                (unsigned long)GIMBAL_TEST_STEP_FREQ_HZ);
        }
        else
        {
            Serial_Printf("[gimbal,x,error,start]\r\n");
        }
        return;
    }

    if (key == 2U)
    {
        if (GimbalStepper_IsBusy(GIMBAL_AXIS_Y) != 0U)
        {
            Serial_Printf("[gimbal,y,busy]\r\n");
        }
        else if (GimbalStepper_StartMove(GIMBAL_AXIS_Y,
                     GIMBAL_DIR_POSITIVE,
                     GIMBAL_TEST_PULSES, GIMBAL_TEST_STEP_FREQ_HZ))
        {
            Serial_Printf("[gimbal,y,start,%lu,%lu]\r\n",
                (unsigned long)GIMBAL_TEST_PULSES,
                (unsigned long)GIMBAL_TEST_STEP_FREQ_HZ);
        }
        else
        {
            Serial_Printf("[gimbal,y,error,start]\r\n");
        }
        return;
    }

    if (key == 3U)
    {
        /* K3 release also calls StopAll through App_CheckK3Emergency above. */
        return;
    }

    /* K4: no operation in step test mode. */
}
