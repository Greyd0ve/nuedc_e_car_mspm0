#ifndef __APP_VISUAL_GIMBAL_H
#define __APP_VISUAL_GIMBAL_H

#include <stdint.h>

typedef enum
{
    VISUAL_GIMBAL_WAIT_LINK = 0,
    VISUAL_GIMBAL_WAIT_FRAME,
    VISUAL_GIMBAL_WAIT_NEW_FRAME,
    VISUAL_GIMBAL_DEADBAND,
    VISUAL_GIMBAL_MOVING,
    VISUAL_GIMBAL_INVALID,
    VISUAL_GIMBAL_FAULT,
    VISUAL_GIMBAL_LIMIT
} VisualGimbalState_t;

typedef struct
{
    VisualGimbalState_t state;
    uint16_t sequence;
    int16_t  rawErrorX;
    int16_t  filteredErrorX;
    uint8_t  errorValid;
    uint8_t  filterSampleCount;
    int8_t   direction;
    uint32_t commandPulses;
    uint32_t frequencyHz;
    int32_t  estimatedPositionPulses;
    uint32_t acceptedCommands;
    uint32_t deadbandFrames;
    uint32_t invalidFrames;
    uint32_t busySkips;
    uint32_t limitRejects;
    uint32_t staleFrames;
    uint32_t startFailures;
    uint32_t skippedControlFrames;
    uint32_t directionChanges;
    uint32_t maxObservationAgeMs;
    uint32_t processedValidFrames;
    uint8_t  commandAccepted;
    uint8_t  locked;
    uint8_t  lockConfirmCount;
    uint32_t lockEnterCount;
    uint8_t  faultLatched;
    uint8_t  positionValid;
} VisualGimbalDebug_t;

void VisualGimbal_Init(void);
void VisualGimbal_Task10ms(void);
void VisualGimbal_Stop(void);
void VisualGimbal_GetDebug(VisualGimbalDebug_t *outDebug);

#endif
