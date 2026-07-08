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
#include "Servo.h"
#include "Timer.h"
#include "cmsis_compiler.h"
#include <stdint.h>

volatile uint8_t g_task_1ms_count = 0U;
volatile uint8_t g_task_5ms_count = 0U;
volatile uint8_t g_task_10ms_count = 0U;
volatile uint8_t g_task_100ms_count = 0U;
volatile uint8_t g_task_200ms_count = 0U;

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

int main(void)
{
    SYSCFG_DL_init();

    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Motor_StopAll();
    Encoder_Init();
    BeepLed_Init();
    Serial_Init();
    Servo_Init();
    App_Control_Init();
    ECar_Init();
    ECar_Serial_Init();

#if ECAR_TEST_OLED_ENABLE || !ECAR_BOARD_TEST_MODE
    OLED_Init();
    OLED_Clear();
#endif

#if ECAR_TEST_IMU_ENABLE
    IMU_Init();
#endif

#if ECAR_BOARD_TEST_MODE
    BoardTest_Init();
#else
    App_Line_GPIOForceInit();
    ECar_ShowStatus();
#endif

    Timer_Init();

    while (1)
    {
        uint8_t taskCount;

        (void)Main_TakeTaskCounterAll(&g_task_1ms_count);

        taskCount = Main_TakeTaskCounterAll(&g_task_5ms_count);
        if (taskCount > 0U)
        {
            App_Control_UpdateEncoderSpeed((uint16_t)taskCount * ECAR_ENCODER_SPEED_PERIOD_MS);
        }

        taskCount = Main_TakeTaskCounterAll(&g_task_10ms_count);
        if (taskCount > 2U)
        {
            taskCount = 2U;
        }
        while (taskCount > 0U)
        {
#if ECAR_BOARD_TEST_MODE
            BoardTest_Task10ms();
#else
            ECar_Control10ms();
#endif
            taskCount--;
        }

#if !ECAR_BOARD_TEST_MODE
        ECar_KeyProcess();
#endif
        ECar_SerialProcess();

        if (Main_TakeTaskCounterAll(&g_task_100ms_count) > 0U)
        {
#if ECAR_BOARD_TEST_MODE
            BoardTest_Task100ms();
#else
            ECar_SerialPlot100ms();
#endif
        }

        if (Main_TakeTaskCounterAll(&g_task_200ms_count) > 0U)
        {
#if ECAR_BOARD_TEST_MODE
            BoardTest_Task200ms();
#else
            ECar_ShowStatus();
#endif
        }
    }
}
