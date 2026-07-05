#include "ti_msp_dl_config.h"
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
#include <stdint.h>

volatile uint8_t g_flag_10ms = 0U;
volatile uint8_t g_oledRefreshFlag = 0U;

int main(void)
{
    SYSCFG_DL_init();

    OLED_Init();
    Key_Init();
    Grayscale_Init();
    Motor_Init();
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
        if (g_flag_10ms)
        {
            g_flag_10ms = 0U;
            ECar_Control10ms();
            ECar_SerialPlot10ms();
        }

        ECar_KeyProcess();
        ECar_SerialProcess();

        if (g_oledRefreshFlag)
        {
            g_oledRefreshFlag = 0U;
            ECar_ShowStatus();
        }
    }
}
