#include "app_aim_link.h"
#include "app_e_car.h"
#include "Serial.h"
#include <stdint.h>

void AimLink_Init(void)
{
    /* Reserved for a dedicated K230 UART or packet bridge. */
}

void AimLink_Task100ms(void)
{
    /* Intentionally empty until the K230 link is assigned to real hardware. */
}

void AimLink_SendCarState(void)
{
    ECarState_t state;
    uint8_t progress;

    state = ECar_GetState();
    progress = (uint8_t)(ECar_GetLapProgress() * 100.0f);
    if (progress > 99U)
    {
        progress = 99U;
    }

    switch (state)
    {
        case E_CAR_IDLE:
        case E_CAR_READY:
            Serial_Printf("[car,state,ready,n,%u]\r\n",
                          (unsigned int)ECar_GetTargetLap());
            break;

        case E_CAR_LINE_RUN:
        case E_CAR_CORNER_ENTER:
        case E_CAR_CORNER_TURN:
        case E_CAR_LINE_RECOVER:
            Serial_Printf("[car,state,run,lap,%u,corner,%u,progress,%u,time,%lu]\r\n",
                          (unsigned int)ECar_GetLapCount(),
                          (unsigned int)ECar_GetCornerCount(),
                          (unsigned int)progress,
                          (unsigned long)ECar_GetRunningTimeMs());
            break;

        case E_CAR_FINISH:
            Serial_Printf("[car,state,finish,lap,%u]\r\n",
                          (unsigned int)ECar_GetLapCount());
            break;

        case E_CAR_FAULT:
        default:
            Serial_Printf("[car,state,fault,code,%u]\r\n",
                          (unsigned int)ECar_GetFaultCode());
            break;
    }
}

void AimLink_OnAimPacket(const char *packet)
{
    (void)packet;
    /* Parse-only entry point reserved. K230 packets must not start the car or drive motors. */
}
