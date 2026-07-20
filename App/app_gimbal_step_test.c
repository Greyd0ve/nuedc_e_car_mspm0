#include "app_gimbal_step_test.h"
#include "app_config.h"
#include "GimbalStepper.h"
#include "Key.h"
#include "Serial.h"
#include "Board_Config.h"
#include <stdint.h>

static uint8_t s_initDone = 0U;

static uint8_t App_CheckDone(void)
{
    uint8_t reported = 0U;

    if (GimbalStepper_IsDone(GIMBAL_AXIS_X))
    {
        GimbalStepper_ClearDone(GIMBAL_AXIS_X);
        Serial_Printf("[gimbal,x,done]\r\n");
        reported = 1U;
    }

    if (GimbalStepper_IsDone(GIMBAL_AXIS_Y))
    {
        GimbalStepper_ClearDone(GIMBAL_AXIS_Y);
        Serial_Printf("[gimbal,y,done]\r\n");
        reported = 1U;
    }

    (void)reported;
    return reported;
}

void GimbalStepTest_Init(void)
{
    s_initDone = 1U;
    GimbalStepper_Init();
}

void GimbalStepTest_KeyProcess(void)
{
    uint8_t key;

    if (s_initDone == 0U) { return; }

    App_CheckDone();

    key = Key_GetNum();
    if (key == 0U) { return; }

    if (key == 1U)
    {
        if (GimbalStepper_IsBusy(GIMBAL_AXIS_X) == 0U)
        {
            GimbalStepper_StartMove(GIMBAL_AXIS_X,
                (int8_t)GIMBAL_X_POSITIVE_DIR_LEVEL,
                GIMBAL_TEST_PULSES, GIMBAL_TEST_STEP_FREQ_HZ);
            Serial_Printf("[gimbal,x,start,%lu,%lu]\r\n",
                (unsigned long)GIMBAL_TEST_PULSES,
                (unsigned long)GIMBAL_TEST_STEP_FREQ_HZ);
        }
        else
        {
            Serial_Printf("[gimbal,x,busy]\r\n");
        }
        return;
    }

    if (key == 2U)
    {
        if (GimbalStepper_IsBusy(GIMBAL_AXIS_Y) == 0U)
        {
            GimbalStepper_StartMove(GIMBAL_AXIS_Y,
                (int8_t)GIMBAL_Y_POSITIVE_DIR_LEVEL,
                GIMBAL_TEST_PULSES, GIMBAL_TEST_STEP_FREQ_HZ);
            Serial_Printf("[gimbal,y,start,%lu,%lu]\r\n",
                (unsigned long)GIMBAL_TEST_PULSES,
                (unsigned long)GIMBAL_TEST_STEP_FREQ_HZ);
        }
        else
        {
            Serial_Printf("[gimbal,y,busy]\r\n");
        }
        return;
    }

    if (key == 3U)
    {
        GimbalStepper_StopAll();
        Serial_Printf("[gimbal,stop,all]\r\n");
        return;
    }

    /* K4: no operation in step test mode. */
}
