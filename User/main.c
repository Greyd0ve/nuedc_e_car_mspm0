#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "app_board_test.h"
#include "app_control.h"
#include "app_e_car.h"
#include "app_e_serial.h"
#include "app_line.h"
#include "BeepLed.h"
#include "Encoder.h"
#include "Grayscale.h"
#include "IMU.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Serial.h"
#include "Timer.h"
#include "cmsis_compiler.h"
#include <stdint.h>

#if ECAR_AIM_LINK_TEST_MODE
#include "app_aim_link.h"
#include "K230Uart.h"
#elif ECAR_GIMBAL_STEP_TEST_MODE
#include "GimbalStepper.h"
#include "app_gimbal_step_test.h"
#endif

volatile uint8_t g_task_1ms_count = 0U;
volatile uint8_t g_task_5ms_count = 0U;
volatile uint8_t g_task_10ms_count = 0U;
volatile uint8_t g_task_100ms_count = 0U;
volatile uint8_t g_task_200ms_count = 0U;

/*
 * mspm0g3507.sct places this section at the end of flash with +Last.
 * Keep this section, but keep it small to avoid MDK Lite 32KB limit.
 */
__attribute__((used, section(".rodata.flash_tail_pad")))
static const uint8_t s_flashTailPadForKeilFlm[0x10] = { 0U };

static uint8_t Main_TakeTaskCounterAll(volatile uint8_t *counter)
{
    uint8_t count;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    count = *counter;
    *counter = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return count;
}

#if ECAR_ENCODER_MINIMAL_DEBUG
static void Main_PrintfSingleFieldTest(void)
{
    Serial_Printf("[printf-test]\r\n");
    Serial_Printf("risr=%lu\r\n", (unsigned long)Encoder_GetRightIsrCount());
    Serial_Printf("rign=%lu\r\n", (unsigned long)Encoder_GetRightSameAIgnored());
    Serial_Printf("rstat=%lu\r\n", (unsigned long)Encoder_GetRightStatusCount());
    Serial_Printf("rraw=%ld\r\n", (long)Encoder_GetRightLastRawDeltaBeforeLimit());
    Serial_Printf("rlim=%lu\r\n", (unsigned long)Encoder_GetRightLimitHitCount());
    Serial_Printf("rget=%lu\r\n", (unsigned long)Encoder_GetRightGetDeltaCount());
    Serial_Printf("rnz=%lu\r\n", (unsigned long)Encoder_GetRightNonZeroGetCount());
    Serial_Printf("rmax=%ld\r\n", (long)Encoder_GetRightMaxRawDelta());
}
#endif

#if ECAR_AIM_LINK_TEST_MODE
static void Main_PrintAimDebug500ms(void)
{
    AimObservation_t obs;
    AimProtocolStats_t stats;
    int16_t errX, errY;
    uint32_t age;

    AimLink_GetProtocolStats(&stats);

    if (AimLink_GetLatestObservation(&obs))
    {
        age = AimLink_GetObservationAgeMs();

        Serial_Printf("[aim,ok,%lu,crc,%lu,seq,%u,state,%u,flags,%02X",
            (unsigned long)stats.validFrames,
            (unsigned long)stats.crcErrors,
            (unsigned int)obs.sequence,
            (unsigned int)obs.trackingState,
            (unsigned int)obs.validFlags);

        if ((obs.validFlags & AIM_FLAG_RECT_VALID) && (obs.validFlags & AIM_FLAG_LASER_VALID))
        {
            errX = (int16_t)obs.rectX - (int16_t)obs.laserX;
            errY = (int16_t)obs.rectY - (int16_t)obs.laserY;
            Serial_Printf(",rect,%u,%u,laser,%u,%u,err,%d,%d",
                (unsigned int)obs.rectX, (unsigned int)obs.rectY,
                (unsigned int)obs.laserX, (unsigned int)obs.laserY,
                (int)errX, (int)errY);
        }
        else
        {
            Serial_Printf(",err,NA");
        }

        Serial_Printf(",age,%lu,drop,%lu,dup,%lu,old,%lu,ovf,%lu]\r\n",
            (unsigned long)age,
            (unsigned long)stats.droppedFrames,
            (unsigned long)stats.duplicateFrames,
            (unsigned long)stats.outOfOrderFrames,
            (unsigned long)K230Uart_GetOverflowCount());
    }
    else
    {
        Serial_Printf("[aim,no_signal,crc,%lu,ovf,%lu]\r\n",
            (unsigned long)stats.crcErrors,
            (unsigned long)K230Uart_GetOverflowCount());
    }
}
#endif

int main(void)
{
    SYSCFG_DL_init();

#if ECAR_ENCODER_MINIMAL_DEBUG
    Serial_Init();
    Encoder_Init();
    Encoder_DebugPrintDirectNoPrintf("[enc-direct-before-timer]");
    Timer_Init();
    Encoder_DebugPrintDirectNoPrintf("[enc-direct-after-timer]");

    while (1)
    {
        static uint16_t printMs = 0U;
        uint8_t taskCount = Main_TakeTaskCounterAll(&g_task_1ms_count);

        while (taskCount > 0U)
        {
            taskCount--;
            printMs++;
            if (printMs >= 1000U)
            {
                printMs = 0U;
                Encoder_DebugPrintDirectNoPrintf("[enc-direct-before-getdelta]");

								{
										int16_t rd = Encoder_GetRightDelta();
										Serial_Printf("[getdelta-test]\r\n");
										Serial_Printf("rd=%d\r\n", (int)rd);
								}

								Encoder_DebugPrintDirectNoPrintf("[enc-direct-after-getdelta]");
								Encoder_DebugPrintGetterNoPrintf("[enc-getter-after-getdelta]");
								Main_PrintfSingleFieldTest();
            }
        }
    }
#else
    Key_Init();
    Motor_Init();
    Motor_StopAll();
    Serial_Init();

#if ECAR_AIM_LINK_TEST_MODE
    AimLink_Init();
#elif ECAR_GIMBAL_STEP_TEST_MODE
    GimbalStepTest_Init();
#else
    Grayscale_Init();
    Encoder_Init();
    BeepLed_Init();
    App_Control_Init();
    ECar_Init();
    ECar_Serial_Init();
#endif

#if ECAR_OLED_ENABLE
    OLED_Init();
    OLED_Clear();
#endif

#if ECAR_IMU_ENABLE && !ECAR_GIMBAL_STEP_TEST_MODE && !ECAR_AIM_LINK_TEST_MODE
    IMU_Init();
#endif

#if ECAR_AIM_LINK_TEST_MODE || ECAR_GIMBAL_STEP_TEST_MODE
    /* No additional init */
#elif ECAR_BOARD_TEST_MODE
    BoardTest_Init();
#else
    App_Line_GPIOForceInit();
#if ECAR_OLED_ENABLE
    ECar_ShowStatus();
#endif
#endif

    Timer_Init();

    while (1)
    {
        uint8_t taskCount;

#if ECAR_AIM_LINK_TEST_MODE
        AimLink_Process();
#endif

        (void)Main_TakeTaskCounterAll(&g_task_1ms_count);

        taskCount = Main_TakeTaskCounterAll(&g_task_5ms_count);
#if !ECAR_GIMBAL_STEP_TEST_MODE && !ECAR_AIM_LINK_TEST_MODE
        if (taskCount > 0U)
        {
            App_Control_UpdateEncoderSpeed((uint16_t)taskCount * ECAR_ENCODER_SPEED_PERIOD_MS);
        }
#endif

        taskCount = Main_TakeTaskCounterAll(&g_task_10ms_count);
        if (taskCount > 2U)
        {
            taskCount = 2U;
        }
        while (taskCount > 0U)
        {
#if ECAR_AIM_LINK_TEST_MODE || ECAR_GIMBAL_STEP_TEST_MODE
            /* No car control */
#elif ECAR_BOARD_TEST_MODE
            BoardTest_Task10ms();
#else
            ECar_Control10ms();
#endif
            taskCount--;
        }

#if ECAR_AIM_LINK_TEST_MODE
        /* Key and serial not used in aim link test mode */
#elif ECAR_GIMBAL_STEP_TEST_MODE
        GimbalStepTest_KeyProcess();
#elif !ECAR_BOARD_TEST_MODE
        ECar_KeyProcess();
#endif

#if !ECAR_GIMBAL_STEP_TEST_MODE && !ECAR_AIM_LINK_TEST_MODE
        ECar_SerialProcess();
#endif

        if (Main_TakeTaskCounterAll(&g_task_100ms_count) > 0U)
        {
#if ECAR_AIM_LINK_TEST_MODE || ECAR_GIMBAL_STEP_TEST_MODE
            /* No 100ms task */
#elif ECAR_BOARD_TEST_MODE
            BoardTest_Task100ms();
#else
            ECar_SerialPlot100ms();
#endif
        }

        if (Main_TakeTaskCounterAll(&g_task_200ms_count) > 0U)
        {
#if ECAR_AIM_LINK_TEST_MODE
            Main_PrintAimDebug500ms();
#elif ECAR_GIMBAL_STEP_TEST_MODE
            /* No 200ms task */
#elif ECAR_BOARD_TEST_MODE
            BoardTest_Task200ms();
#elif ECAR_OLED_ENABLE
            ECar_ShowStatus();
#endif
        }
    }
#endif
}
