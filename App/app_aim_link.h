#ifndef __APP_AIM_LINK_H
#define __APP_AIM_LINK_H

#include "app_aim_protocol.h"

void AimLink_Init(void);
void AimLink_Process(void);

uint8_t AimLink_GetLatestObservation(AimObservation_t *outObservation);
uint32_t AimLink_GetObservationAgeMs(void);
uint8_t AimLink_IsFresh(uint32_t timeoutMs);
void AimLink_GetProtocolStats(AimProtocolStats_t *outStats);

#endif
