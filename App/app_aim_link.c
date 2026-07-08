#include "app_aim_link.h"
#include "app_e_car.h"
#include "Serial.h"
#include <stdint.h>

/* 后续 K230 瞄准模块通信预留接口。
 * 本模块故意不启动电机、不修改目标速度、不解析运动命令；
 * 只为后续瞄准模块数据提供清晰边界。
 */
void AimLink_Init(void)
{
    /* 预留给后续独立 K230 UART 或转发桥初始化。 */
}

void AimLink_Task100ms(void)
{
    /* 当前未绑定硬件链路，保持空实现。 */
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

    /* 文本帧保持简单，方便 K230 侧固件解析。 */
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
    /* 仅预留解析入口；K230 数据不得启动小车或直接驱动电机。 */
}
