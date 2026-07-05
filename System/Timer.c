#include "Timer.h"
#include "Board_Config.h"
#include "Key.h"
#include "app_e_car.h"
#include <stdint.h>

#define CONTROL_PERIOD_MS      10U
#define OLED_REFRESH_PERIOD_MS 200U

extern volatile uint8_t g_flag_10ms;
extern volatile uint8_t g_oledRefreshFlag;

static volatile uint16_t s_oledRefreshMs = 0U;

void Timer_Init(void)
{
    s_oledRefreshMs = 0U;
    DL_TimerG_clearInterruptStatus(SYSTEM_TIMER_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(SYSTEM_TIMER_IRQN);
    NVIC_EnableIRQ(SYSTEM_TIMER_IRQN);
    DL_TimerG_startCounter(SYSTEM_TIMER_INST);
}

void TIMG8_IRQHandler(void)
{
    static uint8_t controlDiv = 0U;

    switch (DL_TimerG_getPendingInterrupt(SYSTEM_TIMER_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            Key_Tick();
            ECar_PromptTick1ms();

            controlDiv++;
            if (controlDiv >= CONTROL_PERIOD_MS)
            {
                controlDiv = 0U;
                g_flag_10ms = 1U;
            }

            if (s_oledRefreshMs < OLED_REFRESH_PERIOD_MS)
            {
                s_oledRefreshMs++;
            }
            else
            {
                s_oledRefreshMs = 0U;
                g_oledRefreshFlag = 1U;
            }
            break;

        default:
            break;
    }
}
