#include "BeepLed.h"
#include "Board_Config.h"

static volatile uint16_t s_beepRemainMs = 0U;

void Beep_On(void)
{
    DL_GPIO_setPins(BEEP_PORT, BEEP_PIN);
}

void Beep_Off(void)
{
    DL_GPIO_clearPins(BEEP_PORT, BEEP_PIN);
}

void Beep_BeepMs(uint16_t ms)
{
    s_beepRemainMs = ms;
    if (ms > 0U)
    {
        Beep_On();
    }
    else
    {
        Beep_Off();
    }
}

void BeepLed_Tick1ms(void)
{
    if (s_beepRemainMs == 0U)
    {
        return;
    }

    s_beepRemainMs--;
    if (s_beepRemainMs == 0U)
    {
        Beep_Off();
    }
}

void LED_User_On(void)
{
    DL_GPIO_setPins(LED_USER_PORT, LED_USER_PIN);
}

void LED_User_Off(void)
{
    DL_GPIO_clearPins(LED_USER_PORT, LED_USER_PIN);
}

void LED_User_Toggle(void)
{
    DL_GPIO_togglePins(LED_USER_PORT, LED_USER_PIN);
}

void BeepLed_Init(void)
{
    s_beepRemainMs = 0U;
    Beep_Off();
    LED_User_Off();
}

void BeepLed_AllOn(void)
{
    Beep_On();
    LED_User_On();
}

void BeepLed_AllOff(void)
{
    s_beepRemainMs = 0U;
    Beep_Off();
    LED_User_Off();
}

void BeepLed_AllTurn(void)
{
    DL_GPIO_togglePins(BEEP_PORT, BEEP_PIN);
    LED_User_Toggle();
}
