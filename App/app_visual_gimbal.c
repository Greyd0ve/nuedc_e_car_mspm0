#include "app_visual_gimbal.h"
#include "app_config.h"
#include "app_aim_link.h"
#include "GimbalStepper.h"
#include "Timer.h"
#include "cmsis_compiler.h"

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
static uint8_t  s_commandAccepted;

static uint32_t s_staleFrames;
static uint32_t s_startFailures;
static uint32_t s_skippedControlFrames;
static uint32_t s_directionChanges;
static uint32_t s_maxObservationAgeMs;
static uint32_t s_processedValidFrames;

static int8_t   s_lastMoveDirection;

static uint8_t  s_faultLatched;
static uint8_t  s_positionValid;

static int16_t s_errorHistory[3];
static uint8_t s_errorHistoryCount;
static uint8_t s_errorHistoryIndex;

static uint8_t  s_locked;
static uint8_t  s_lockConfirmCount;
static uint32_t s_lockEnterCount;

static uint8_t  s_hasLastStaleSequence;
static uint16_t s_lastStaleSequence;

static void VisualGimbal_ResetFilterAndLock(void)
{
    s_errorHistoryCount = 0U;
    s_errorHistoryIndex = 0U;
    s_locked            = 0U;
    s_lockConfirmCount  = 0U;
}

static void VisualGimbal_LatchFault(void)
{
    uint8_t wasBusy = GimbalStepper_IsBusy(GIMBAL_AXIS_X);
    GimbalStepper_Stop(GIMBAL_AXIS_X);
    s_faultLatched = 1U;
    s_state        = VISUAL_GIMBAL_FAULT;
    if (wasBusy) { s_positionValid = 0U; }
    VisualGimbal_ResetFilterAndLock();
}

static uint32_t VisualGimbal_ErrorToPulses(int16_t errorX)
{
    uint32_t absErr;
    if (errorX < 0) { absErr = (uint32_t)(-errorX); }
    else            { absErr = (uint32_t)errorX; }

    if (absErr <= (uint32_t)AIM_X_DEADBAND_PX)      { return 0U; }
    else if (absErr <= 12U)                          { return 1U; }
    else if (absErr <= 30U)                          { return 2U; }
    else if (absErr <= 60U)                          { return 4U; }
    else if (absErr <= 120U)                         { return 6U; }
    else                                             { return AIM_X_MAX_SINGLE_PULSES; }
}

static int16_t VisualGimbal_MedianOfThree(int16_t a, int16_t b, int16_t c)
{
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (a > c) { int16_t t = a; a = c; c = t; }
    if (b > c) { int16_t t = b; b = c; c = t; }
    return b;
}

static int16_t VisualGimbal_PushFilter(int16_t rawError)
{
    s_errorHistory[s_errorHistoryIndex] = rawError;
    s_errorHistoryIndex++;
    if (s_errorHistoryIndex >= 3U) { s_errorHistoryIndex = 0U; }
    if (s_errorHistoryCount < 3U)  { s_errorHistoryCount++; }

    if (s_errorHistoryCount == 1U)
    {
        return rawError;
    }
    else if (s_errorHistoryCount == 2U)
    {
        int16_t v0 = s_errorHistory[0];
        int16_t v1 = s_errorHistory[1];
        return (int16_t)(((int32_t)v0 + (int32_t)v1) / 2);
    }
    else
    {
        return VisualGimbal_MedianOfThree(
            s_errorHistory[0], s_errorHistory[1], s_errorHistory[2]);
    }
}

static uint8_t VisualGimbal_IsSequenceNew(uint16_t sequence)
{
    if (!s_hasLastHandledSequence) { return 1U; }
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
    uint8_t i;
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
    s_commandAccepted          = 0U;

    s_staleFrames              = 0U;
    s_startFailures            = 0U;
    s_skippedControlFrames     = 0U;
    s_directionChanges         = 0U;
    s_maxObservationAgeMs      = 0U;
    s_processedValidFrames     = 0U;

    s_lastMoveDirection        = 0;

    s_faultLatched             = 0U;
    s_positionValid            = 1U;

    for (i = 0U; i < 3U; i++) { s_errorHistory[i] = 0; }
    s_errorHistoryCount        = 0U;
    s_errorHistoryIndex        = 0U;

    s_locked                   = 0U;
    s_lockConfirmCount         = 0U;
    s_lockEnterCount           = 0U;

    s_hasLastStaleSequence     = 0U;
    s_lastStaleSequence        = 0U;
}

void VisualGimbal_Task10ms(void)
{
    AimObservation_t obs;
    AimLinkHealth_t  health;
    uint32_t age;
    int16_t rawError, filteredError;
    int8_t  logicalDirection;
    uint32_t pulses;
    int32_t nextPos;
    int32_t dirSign;

    if (s_faultLatched) { s_state = VISUAL_GIMBAL_FAULT; return; }

    health = AimLink_GetHealth();
    if (health == AIM_LINK_FAULT) { VisualGimbal_LatchFault(); return; }

    if (GimbalStepper_IsBusy(GIMBAL_AXIS_X)) { s_busySkips++; return; }

    if (s_state == VISUAL_GIMBAL_MOVING) { s_state = VISUAL_GIMBAL_WAIT_NEW_FRAME; }

    if (health == AIM_LINK_NO_SIGNAL) { s_state = VISUAL_GIMBAL_WAIT_LINK; return; }

    if (!AimLink_GetLatestObservation(&obs)) { s_state = VISUAL_GIMBAL_WAIT_FRAME; return; }

    age = Timer_GetMillis() - obs.rxTimeMs;
    if (age > s_maxObservationAgeMs) { s_maxObservationAgeMs = age; }

    if (age > AIM_XY_DATA_MAX_AGE_MS)
    {
        if (!s_hasLastStaleSequence || obs.sequence != s_lastStaleSequence)
        {
            s_hasLastStaleSequence = 1U;
            s_lastStaleSequence    = obs.sequence;
            s_staleFrames++;
        }
        VisualGimbal_ResetFilterAndLock();
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (obs.trackingState == AIM_TRACK_FAULT) { VisualGimbal_LatchFault(); return; }
    if (obs.trackingState != AIM_TRACK_TRACKING)
    {
        VisualGimbal_RecordInvalidSequence(obs.sequence);
        VisualGimbal_ResetFilterAndLock();
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (!(obs.validFlags & AIM_FLAG_RECT_VALID) ||
        !(obs.validFlags & AIM_FLAG_LASER_VALID) ||
        !(obs.validFlags & AIM_FLAG_MEASUREMENT_FRESH))
    {
        VisualGimbal_RecordInvalidSequence(obs.sequence);
        VisualGimbal_ResetFilterAndLock();
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    if (!s_positionValid) { VisualGimbal_RecordInvalidSequence(obs.sequence); return; }

    if (!VisualGimbal_IsSequenceNew(obs.sequence)) { return; }

    s_processedValidFrames++;

    {
        uint16_t delta = (uint16_t)(obs.sequence - s_lastHandledSequence);
        if (s_hasLastHandledSequence && delta > 1U && delta < 0x8000U)
        {
            s_skippedControlFrames += (uint32_t)(delta - 1U);
        }
    }

    rawError = (int16_t)obs.rectX - (int16_t)obs.laserX;
    filteredError = VisualGimbal_PushFilter(rawError);

    {
        uint32_t absErr = (uint32_t)((filteredError >= 0) ? filteredError : -filteredError);

        if (s_locked)
        {
            if (absErr <= (uint32_t)AIM_X_LOCK_EXIT_PX)
            {
                s_deadbandFrames++;
                s_state = VISUAL_GIMBAL_DEADBAND;
                s_lastDirection     = 0;
                s_lastCommandPulses = 0U;
                s_lastFrequencyHz   = 0U;
                s_commandAccepted   = 0U;
                VisualGimbal_MarkSequenceHandled(obs.sequence);
                return;
            }
            s_locked           = 0U;
            s_lockConfirmCount = 0U;
        }
        else
        {
            if (absErr <= (uint32_t)AIM_X_LOCK_ENTER_PX)
            {
                s_lockConfirmCount++;
                if (s_lockConfirmCount >= AIM_X_LOCK_CONFIRM_FRAMES)
                {
                    s_locked         = 1U;
                    s_lockEnterCount++;
                    s_deadbandFrames++;
                    s_state = VISUAL_GIMBAL_DEADBAND;
                    s_lastDirection     = 0;
                    s_lastCommandPulses = 0U;
                    s_lastFrequencyHz   = 0U;
                    s_commandAccepted   = 0U;
                    VisualGimbal_MarkSequenceHandled(obs.sequence);
                    return;
                }
                s_deadbandFrames++;
                s_state = VISUAL_GIMBAL_DEADBAND;
                s_lastDirection     = 0;
                s_lastCommandPulses = 0U;
                s_lastFrequencyHz   = 0U;
                s_commandAccepted   = 0U;
                VisualGimbal_MarkSequenceHandled(obs.sequence);
                return;
            }
            else
            {
                s_lockConfirmCount = 0U;
            }
        }
    }

    pulses = VisualGimbal_ErrorToPulses(filteredError);

    if (pulses == 0U)
    {
        VisualGimbal_MarkSequenceHandled(obs.sequence);
        s_deadbandFrames++;
        s_state             = VISUAL_GIMBAL_DEADBAND;
        s_lastDirection     = 0;
        s_lastCommandPulses = 0U;
        s_lastFrequencyHz   = 0U;
        s_commandAccepted   = 0U;
        return;
    }

    if (filteredError > 0)
    {
        logicalDirection = (AIM_X_ERROR_POSITIVE_DIR > 0) ? GIMBAL_DIR_POSITIVE : GIMBAL_DIR_NEGATIVE;
    }
    else
    {
        logicalDirection = (AIM_X_ERROR_POSITIVE_DIR > 0) ? GIMBAL_DIR_NEGATIVE : GIMBAL_DIR_POSITIVE;
    }

    dirSign = (logicalDirection > 0) ? 1 : -1;
    nextPos = s_estimatedPositionPulses + dirSign * (int32_t)pulses;

    if (nextPos > (int32_t)AIM_X_SOFT_LIMIT_PULSES ||
        nextPos < -(int32_t)AIM_X_SOFT_LIMIT_PULSES)
    {
        VisualGimbal_MarkSequenceHandled(obs.sequence);
        s_limitRejects++;
        s_state               = VISUAL_GIMBAL_LIMIT;
        s_lastDirection       = logicalDirection;
        s_lastCommandPulses   = pulses;
        s_lastFrequencyHz     = 0U;
        s_commandAccepted     = 0U;
        return;
    }

    if (s_lastMoveDirection != 0 && s_lastMoveDirection != logicalDirection)
    {
        s_directionChanges++;
    }
    s_lastMoveDirection = logicalDirection;

    if (!GimbalStepper_StartMove(GIMBAL_AXIS_X, logicalDirection, pulses, AIM_X_STEP_FREQ_HZ))
    {
        VisualGimbal_MarkSequenceHandled(obs.sequence);
        s_startFailures++;
        s_state = VISUAL_GIMBAL_INVALID;
        return;
    }

    VisualGimbal_MarkSequenceHandled(obs.sequence);
    s_estimatedPositionPulses = nextPos;
    s_acceptedCommands++;
    s_state               = VISUAL_GIMBAL_MOVING;
    s_lastDirection       = logicalDirection;
    s_lastCommandPulses   = pulses;
    s_lastFrequencyHz     = AIM_X_STEP_FREQ_HZ;
    s_commandAccepted     = 1U;
}

void VisualGimbal_Stop(void)
{
    VisualGimbal_LatchFault();
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
    outDebug->commandAccepted         = s_commandAccepted;

    outDebug->staleFrames             = s_staleFrames;
    outDebug->startFailures           = s_startFailures;
    outDebug->skippedControlFrames    = s_skippedControlFrames;
    outDebug->directionChanges        = s_directionChanges;
    outDebug->maxObservationAgeMs     = s_maxObservationAgeMs;
    outDebug->processedValidFrames    = s_processedValidFrames;
    outDebug->faultLatched            = s_faultLatched;
    outDebug->positionValid           = s_positionValid;
    outDebug->filterSampleCount       = s_errorHistoryCount;
    outDebug->locked                  = s_locked;
    outDebug->lockConfirmCount        = s_lockConfirmCount;
    outDebug->lockEnterCount          = s_lockEnterCount;

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
                outDebug->rawErrorX  = (int16_t)rx - (int16_t)lx;
            }
            else
            {
                outDebug->errorValid = 0U;
                outDebug->rawErrorX  = 0;
            }
        }
        else
        {
            outDebug->errorValid = 0U;
            outDebug->rawErrorX  = 0;
        }
    }
    else
    {
        outDebug->sequence   = 0U;
        outDebug->errorValid = 0U;
        outDebug->rawErrorX  = 0;
    }

    outDebug->filteredErrorX = (s_errorHistoryCount > 0U)
        ? s_errorHistory[(s_errorHistoryIndex > 0U) ? (s_errorHistoryIndex - 1U) : 2U] : 0;

    if (primask == 0U) { __enable_irq(); }
}
