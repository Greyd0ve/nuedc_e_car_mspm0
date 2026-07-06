#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "Grayscale.h"
#include "BeepLed.h"
#include "app_control.h"
#include "app_line.h"
#include "app_e_car.h"
#include "app_e_serial.h"
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

    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
    Motor_StopAll();
    Encoder_Init();
    App_Line_GPIOForceInit();
    BeepLed_Init();
    Serial_Init();
    App_Control_Init();
    ECar_Init();
    ECar_Serial_Init();
    Timer_Init();

    OLED_Clear();
    ECar_ShowStatus();

    while (1)
    {
        uint8_t taskCount;

        if (Main_TakeTaskCounterAll(&g_task_1ms_count) > 0U)
        {
            /* Reserved for light 1 ms services. Full grayscale processing stays out of ISR. */
        }

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
            App_Line_Update();
#else
            ECar_Control10ms();
#endif
            taskCount--;
        }

        ECar_KeyProcess();
        ECar_SerialProcess();

        if (Main_TakeTaskCounterAll(&g_task_100ms_count) > 0U)
        {
            ECar_SerialPlot100ms();
        }

        if (Main_TakeTaskCounterAll(&g_task_200ms_count) > 0U)
        {
            ECar_ShowStatus();
        }
    }
}
