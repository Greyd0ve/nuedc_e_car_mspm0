#ifndef __GIMBAL_STEPPER_H
#define __GIMBAL_STEPPER_H

#include <stdint.h>

typedef enum
{
    GIMBAL_AXIS_X = 0,
    GIMBAL_AXIS_Y,
    GIMBAL_AXIS_COUNT
} GimbalAxis_t;

/*
 * logicalDirection: +1 = logical positive, -1 = logical negative.
 * All other values are rejected.
 * The actual DIR pin level is determined by per-axis positiveDirLevel macro.
 */
#define GIMBAL_DIR_POSITIVE  ((int8_t)(+1))
#define GIMBAL_DIR_NEGATIVE  ((int8_t)(-1))

void GimbalStepper_Init(void);

uint8_t GimbalStepper_StartMove(
    GimbalAxis_t axis,
    int8_t       logicalDirection,
    uint32_t     pulseCount,
    uint32_t     frequencyHz);

void GimbalStepper_Stop(GimbalAxis_t axis);
void GimbalStepper_StopAll(void);
uint8_t GimbalStepper_IsBusy(GimbalAxis_t axis);
uint32_t GimbalStepper_GetRemainingPulses(GimbalAxis_t axis);

uint8_t GimbalStepper_IsDone(GimbalAxis_t axis);
void GimbalStepper_ClearDone(GimbalAxis_t axis);

#endif
