#ifndef __APP_VISUAL_GIMBAL_H
#define __APP_VISUAL_GIMBAL_H

#include <stdint.h>

typedef enum
{
    VISUAL_GIMBAL_WAIT_LINK = 0,
    VISUAL_GIMBAL_WAIT_FRAME,
    VISUAL_GIMBAL_DEADBAND,
    VISUAL_GIMBAL_MOVING,
    VISUAL_GIMBAL_INVALID,
    VISUAL_GIMBAL_LIMIT
} VisualGimbalState_t;

typedef struct
{
    VisualGimbalState_t state;
    uint16_t sequence;
    int16_t  errorX;
    int8_t   direction;
    uint32_t commandPulses;
    uint32_t frequencyHz;
    int32_t  estimatedPositionPulses;
    uint32_t acceptedCommands;
    uint32_t deadbandFrames;
    uint32_t invalidFrames;
    uint32_t busySkips;
    uint32_t limitRejects;
} VisualGimbalDebug_t;

void VisualGimbal_Init(void);
void VisualGimbal_Task10ms(void);
void VisualGimbal_Stop(void);
void VisualGimbal_GetDebug(VisualGimbalDebug_t *outDebug);

#endif
