#include "app_aim_link.h"
#include "K230Uart.h"
#include "Timer.h"
#include <stdint.h>

void AimLink_Init(void)
{
    K230Uart_Init();
    AimProtocol_Init();
}

void AimLink_Process(void)
{
    AimProtocol_Process();
}

uint8_t AimLink_GetLatestObservation(AimObservation_t *outObservation)
{
    return AimProtocol_GetLatest(outObservation);
}

uint32_t AimLink_GetObservationAgeMs(void)
{
    AimObservation_t obs;

    if (!AimProtocol_GetLatest(&obs))
    {
        return 0xFFFFFFFFUL;
    }

    return Timer_GetMillis() - obs.rxTimeMs;
}

uint8_t AimLink_IsFresh(uint32_t timeoutMs)
{
    uint32_t age = AimLink_GetObservationAgeMs();

    if (age == 0xFFFFFFFFUL) { return 0U; }
    return (uint8_t)((age <= timeoutMs) ? 1U : 0U);
}

void AimLink_GetProtocolStats(AimProtocolStats_t *outStats)
{
    AimProtocol_GetStats(outStats);
}
