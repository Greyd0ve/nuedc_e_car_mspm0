#include "app_visual_gimbal.h"
#include "app_config.h"
#include "app_aim_link.h"
#include "GimbalStepper.h"
#include "Timer.h"
#include "cmsis_compiler.h"

#define AIM_DATA_MAX_AGE_MS       200U

static VisualGimbalState_t s_state;
static int32_t  s_estimatedPositionPulses;
static uint8_t  s_hasLastHandledSequence;
static uint16_t s_lastHandledSequence;
static uint32_t s_acceptedCommands;
static uint32_t s_deadbandFrames;
static uint32_t s_invalidFrames;
static uint32_t s_busySkips;
static uint32_t s_limitRejects;

static uint8_t  s_hasLastInvalidSequence;
static uint16_t s_lastInvalidSequence;

static int8_t   s_lastDirection;
static uint32_t s_lastCommandPulses;
static uint32_t s_lastFrequencyHz;

static uint32_t VisualGimbal_ErrorToPulses(int16_t errorX)
{
    uint32_t absErr;

    if (errorX < 0) { absErr = (uint32_t)(-errorX); }
    else            { absErr = (uint32_t)errorX; }

    if (absErr <= (uint32_t)AIM_X_DEADBAND_PX)         { return 0U; }
    else if (absErr <= 15U)                             { return 1U; }
    else if (absErr <= 40U)                             { return 3U; }
    else if (absErr <= 80U)                             { return 6U; }
    else if (absErr <= 160U)                            { return 12U; }
    else                                                { return AIM_X_MAX_SINGLE_PULSES; }
}

static uint8_t VisualGimbal_IsSequenceNew(uint16_t sequence)
{
    if (!s_hasLastHandledSequence)
    {
        return 1U;
    }
    return (uint8_t)((sequence != s_lastHandledSequence) ? 1U : 0U);
}

static void VisualGimbal_MarkSequenceHandled(uint16_t sequence)
{
    s_hasLastHandledSequence = 1U;
    s_lastHandledSequence    = sequence;
}

static void VisualGimbal_RecordInvalidSequence(uint16_t sequence)
{
    if (!s_hasLastInvalidSequence || sequence != s_lastInvalidSequence)
    {
        s_hasLastInvalidSequence = 1U;
        s_lastInvalidSequence    = sequence;
        s_invalidFrames++;
    }
}

void VisualGimbal_Init(void)
{
    s_state                    = VISUAL_GIMBAL_WAIT_LINK;
    s_estimatedPositionPulses  = 0;
    s_hasLastHandledSequence   = 0U;
    s_lastHandledSequence      = 0U;
    s_acceptedCommands         = 0U;
    s_deadbandFrames           = 0U;
    s_invalidFrames            = 0U;
    s_busySkips                = 0U;
    s_limitRejects             = 0U;

    s_hasLastInvalidSequence   = 0U;
    s_lastInvalidSequence      = 0U;

    s_lastDirection            = 0;
    s_lastCommandPulses        = 0U;
    s_lastFrequencyHz          = 0U;
}

void VisualGimbal_Task10ms(void)
{
    AimObservation_t obs;
    AimLinkHealth_t  health;
    uint32_t age;

    health = AimLink_GetHealth();

    if (health == AIM_LINK_FAULT)
    {
        GimbalStepper_Stop(GIMBAL_AXIS_X);
        s_state = VISUAL_GIMBAL_FAULT;
        return;
    }

    if (GimbalStepper_IsBusy(GIMBAL_AXIS_X))
    {
        s_busySkips++;
        return;
    }

    if (s_state == VISUAL_GIMBAL_MOVING)
    {
        s_state = VISUAL_GIMBAL_WAIT_NEW_FRAME;
    }

    if (health == AIM_LINK_NO_SIGNAL)
    {
        s_state = VISUAL_GIMBAL_WAIT_LINK;
        return;
    }

    if (!AimLink_GetLatestObservation(&obs))
    {
        s_state = VISUAL_GIMBAL_WAIT_FRAME;
        return;
    }

    age = Timer_GetMillis() - obs.rxTimeMs;
    if (age > AIM_DATA_MAX_AGE_MS)
    {
        VisualGimbal_RecordInvalidSequence(obs.sequence);
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (obs.trackingState == AIM_TRACK_FAULT)
    {
        GimbalStepper_Stop(GIMBAL_AXIS_X);
        s_state = VISUAL_GIMBAL_FAULT;
        return;
    }

    if (obs.trackingState != AIM_TRACK_TRACKING)
    {
        VisualGimbal_RecordInvalidSequence(obs.sequence);
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (!(obs.validFlags & AIM_FLAG_RECT_VALID) ||
        !(obs.validFlags & AIM_FLAG_LASER_VALID) ||
        !(obs.validFlags & AIM_FLAG_MEASUREMENT_FRESH))
    {
        VisualGimbal_RecordInvalidSequence(obs.sequence);
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (!VisualGimbal_IsSequenceNew(obs.sequence))
    {
        return;
    }

    {
        int16_t  errorX;
        int8_t   logicalDirection;
        uint32_t pulses;
        int32_t  nextPos;
        int32_t  dirSign;

        errorX = (int16_t)obs.rectX - (int16_t)obs.laserX;
        pulses = VisualGimbal_ErrorToPulses(errorX);

        if (pulses == 0U)
        {
            VisualGimbal_MarkSequenceHandled(obs.sequence);
            s_deadbandFrames++;
            s_state = VISUAL_GIMBAL_DEADBAND;
            s_lastDirection     = 0;
            s_lastCommandPulses = 0U;
            s_lastFrequencyHz   = 0U;
            return;
        }

        if (errorX > 0)
        {
            logicalDirection = (AIM_X_ERROR_POSITIVE_DIR > 0)
                ? GIMBAL_DIR_POSITIVE : GIMBAL_DIR_NEGATIVE;
        }
        else
        {
            logicalDirection = (AIM_X_ERROR_POSITIVE_DIR > 0)
                ? GIMBAL_DIR_NEGATIVE : GIMBAL_DIR_POSITIVE;
        }

        dirSign = (logicalDirection > 0) ? 1 : -1;
        nextPos = s_estimatedPositionPulses + dirSign * (int32_t)pulses;

        if (nextPos > (int32_t)AIM_X_SOFT_LIMIT_PULSES ||
            nextPos < -(int32_t)AIM_X_SOFT_LIMIT_PULSES)
        {
            VisualGimbal_MarkSequenceHandled(obs.sequence);
            s_limitRejects++;
            s_state             = VISUAL_GIMBAL_LIMIT;
            s_lastDirection     = logicalDirection;
            s_lastCommandPulses = pulses;
            s_lastFrequencyHz   = AIM_X_TEST_STEP_FREQ_HZ;
            return;
        }

        if (!GimbalStepper_StartMove(GIMBAL_AXIS_X, logicalDirection,
                pulses, AIM_X_TEST_STEP_FREQ_HZ))
        {
            VisualGimbal_MarkSequenceHandled(obs.sequence);
            s_invalidFrames++;
            s_state = VISUAL_GIMBAL_INVALID;
            return;
        }

        VisualGimbal_MarkSequenceHandled(obs.sequence);
        s_estimatedPositionPulses = nextPos;
        s_acceptedCommands++;
        s_state             = VISUAL_GIMBAL_MOVING;
        s_lastDirection     = logicalDirection;
        s_lastCommandPulses = pulses;
        s_lastFrequencyHz   = AIM_X_TEST_STEP_FREQ_HZ;
    }
}

void VisualGimbal_Stop(void)
{
    GimbalStepper_Stop(GIMBAL_AXIS_X);
    s_state = VISUAL_GIMBAL_FAULT;
}

void VisualGimbal_GetDebug(VisualGimbalDebug_t *outDebug)
{
    AimObservation_t obs;
    uint32_t primask;

    if (outDebug == 0) { return; }

    primask = __get_PRIMASK();
    __disable_irq();

    outDebug->state                   = s_state;
    outDebug->estimatedPositionPulses = s_estimatedPositionPulses;
    outDebug->acceptedCommands        = s_acceptedCommands;
    outDebug->deadbandFrames          = s_deadbandFrames;
    outDebug->invalidFrames           = s_invalidFrames;
    outDebug->busySkips               = s_busySkips;
    outDebug->limitRejects            = s_limitRejects;
    outDebug->direction               = s_lastDirection;
    outDebug->commandPulses           = s_lastCommandPulses;
    outDebug->frequencyHz             = s_lastFrequencyHz;

    if (AimLink_GetLatestObservation(&obs))
    {
        uint8_t flags = obs.validFlags;

        outDebug->sequence = obs.sequence;

        if ((flags & AIM_FLAG_RECT_VALID) && (flags & AIM_FLAG_LASER_VALID))
        {
            uint16_t rx = obs.rectX;
            uint16_t lx = obs.laserX;

            if (rx < AIM_IMAGE_WIDTH && lx < AIM_IMAGE_WIDTH)
            {
                outDebug->errorValid = 1U;
                outDebug->errorX     = (int16_t)rx - (int16_t)lx;
            }
            else
            {
                outDebug->errorValid = 0U;
                outDebug->errorX     = 0;
            }
        }
        else
        {
            outDebug->errorValid = 0U;
            outDebug->errorX     = 0;
        }
    }
    else
    {
        outDebug->sequence   = 0U;
        outDebug->errorValid = 0U;
        outDebug->errorX     = 0;
    }

    if (primask == 0U) { __enable_irq(); }
}
