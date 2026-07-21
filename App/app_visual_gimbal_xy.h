#ifndef __APP_VISUAL_GIMBAL_XY_H
#define __APP_VISUAL_GIMBAL_XY_H

#include <stdint.h>

typedef struct
{
    int16_t  rawError;
    int16_t  filteredError;
    int8_t   direction;
    uint32_t commandPulses;
    uint32_t commandFreqHz;
    uint8_t  commandAccepted;
    int32_t  estimatedPositionPulses;
    uint8_t  positionValid;
    uint8_t  locked;
    uint8_t  lockConfirmCount;
    uint32_t lockEnterCount;
    uint8_t  busy;
    uint32_t acceptedCommands;
    uint32_t deadbandFrames;
    uint32_t busySkips;
    uint32_t limitRejects;
    uint32_t startFailures;
    uint32_t directionChanges;
} VisualAxisDebug_t;

typedef struct
{
    uint16_t sequence;
    uint32_t ageMs;
    uint8_t  globalFaultLatched;
    uint32_t staleFrames;
    uint32_t invalidFrames;
    uint32_t skippedControlFrames;
    VisualAxisDebug_t x;
    VisualAxisDebug_t y;
} VisualGimbalXYDebug_t;

void VisualGimbalXY_Init(void);
void VisualGimbalXY_Task10ms(void);
void VisualGimbalXY_Stop(void);
void VisualGimbalXY_GetDebug(VisualGimbalXYDebug_t *outDebug);

#endif
