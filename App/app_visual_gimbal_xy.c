#include "app_visual_gimbal_xy.h"
#include "app_config.h"
#include "app_aim_link.h"
#include "GimbalStepper.h"
#include "Timer.h"
#include "cmsis_compiler.h"

typedef struct
{
    GimbalAxis_t axis;
    uint32_t     deadbandPx;
    int32_t      softLimitPulses;
    uint32_t     stepFreqHz;
    uint32_t     maxSinglePulses;
    uint32_t     lockEnterPx;
    uint32_t     lockExitPx;
    uint32_t     lockConfirmFrames;
    int8_t       errorPositiveDir;

    int32_t  estimatedPositionPulses;
    uint8_t  positionValid;

    int16_t errorHistory[3];
    uint8_t errorHistoryCount;
    uint8_t errorHistoryIndex;
    int16_t rawError;
    int16_t filteredError;

    uint8_t  locked;
    uint8_t  lockConfirmCount;
    uint32_t lockEnterCount;

    int8_t   lastMoveDirection;
    int8_t   lastCommandDirection;
    uint32_t lastCommandPulses;
    uint32_t lastCommandFreqHz;
    uint8_t  lastCommandAccepted;

    uint32_t acceptedCommands;
    uint32_t deadbandFrames;
    uint32_t busySkips;
    uint32_t limitRejects;
    uint32_t startFailures;
    uint32_t directionChanges;
} VisualGimbalAxis_t;

static VisualGimbalAxis_t s_axisX;
static VisualGimbalAxis_t s_axisY;

static uint8_t  s_globalFaultLatched;
static uint8_t  s_hasLastHandledSequence;
static uint16_t s_lastHandledSequence;
static uint8_t  s_hasLastInvalidSequence;
static uint16_t s_lastInvalidSequence;
static uint32_t s_invalidFrames;
static uint32_t s_staleFrames;
static uint32_t s_skippedControlFrames;
static uint32_t s_hasLastStaleSequence;
static uint16_t s_lastStaleSequence;

static void VisualGimbalXY_ResetAxisFilters(VisualGimbalAxis_t *ax)
{
    uint8_t i;
    for (i = 0U; i < 3U; i++) { ax->errorHistory[i] = 0; }
    ax->errorHistoryCount = 0U;
    ax->errorHistoryIndex = 0U;
    ax->locked            = 0U;
    ax->lockConfirmCount  = 0U;
}

static void VisualGimbalXY_ResetAllFilters(void)
{
    VisualGimbalXY_ResetAxisFilters(&s_axisX);
    VisualGimbalXY_ResetAxisFilters(&s_axisY);
}

static void VisualGimbalXY_LatchFault(void)
{
    uint8_t xWasBusy = GimbalStepper_IsBusy(GIMBAL_AXIS_X);
    uint8_t yWasBusy = GimbalStepper_IsBusy(GIMBAL_AXIS_Y);

    GimbalStepper_Stop(GIMBAL_AXIS_X);
    GimbalStepper_Stop(GIMBAL_AXIS_Y);

    s_globalFaultLatched = 1U;
    if (xWasBusy) { s_axisX.positionValid = 0U; }
    if (yWasBusy) { s_axisY.positionValid = 0U; }
    VisualGimbalXY_ResetAllFilters();
}

static int16_t VisualGimbalXY_MedianOfThree(int16_t a, int16_t b, int16_t c)
{
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (a > c) { int16_t t = a; a = c; c = t; }
    if (b > c) { int16_t t = b; b = c; c = t; }
    return b;
}

static int16_t VisualGimbalXY_PushFilter(VisualGimbalAxis_t *ax, int16_t raw)
{
    ax->errorHistory[ax->errorHistoryIndex] = raw;
    ax->errorHistoryIndex++;
    if (ax->errorHistoryIndex >= 3U) { ax->errorHistoryIndex = 0U; }
    if (ax->errorHistoryCount < 3U)  { ax->errorHistoryCount++; }

    if (ax->errorHistoryCount == 1U) return raw;
    if (ax->errorHistoryCount == 2U)
    {
        return (int16_t)(((int32_t)ax->errorHistory[0] + (int32_t)ax->errorHistory[1]) / 2);
    }
    return VisualGimbalXY_MedianOfThree(
        ax->errorHistory[0], ax->errorHistory[1], ax->errorHistory[2]);
}

static uint32_t VisualGimbalXY_ErrorToPulses(int16_t error, uint32_t deadband, uint32_t maxPulses)
{
    uint32_t absErr = (uint32_t)((error >= 0) ? error : -error);
    if (absErr <= deadband)           return 0U;
    else if (absErr <= 12U)           return 1U;
    else if (absErr <= 30U)           return 2U;
    else if (absErr <= 60U)           return 4U;
    else if (absErr <= 120U)          return 6U;
    else                              return maxPulses;
}

static int8_t VisualGimbalXY_ErrorToDirection(int16_t error, int8_t posDir)
{
    if (error > 0) return (posDir > 0) ? GIMBAL_DIR_POSITIVE : GIMBAL_DIR_NEGATIVE;
    return (posDir > 0) ? GIMBAL_DIR_NEGATIVE : GIMBAL_DIR_POSITIVE;
}

static uint8_t VisualGimbalXY_ProcessAxisLock(VisualGimbalAxis_t *ax, int16_t filteredError,
    uint32_t absErr, uint8_t sequenceIsNew)
{
    if (!sequenceIsNew) return 2U; /* skip */
    ax->deadbandFrames++;

    if (ax->locked)
    {
        if (absErr <= ax->lockExitPx) return 1U; /* locked, no move */
        ax->locked           = 0U;
        ax->lockConfirmCount = 0U;
        return 0U; /* unlocked, proceed */
    }

    if (absErr <= ax->lockEnterPx)
    {
        ax->lockConfirmCount++;
        if (ax->lockConfirmCount >= ax->lockConfirmFrames)
        {
            ax->locked         = 1U;
            ax->lockEnterCount++;
        }
        return 1U; /* lock confirm in progress, no move */
    }
    ax->lockConfirmCount = 0U;
    return 0U; /* proceed */
}

static uint8_t VisualGimbalXY_ProcessAxisMove(VisualGimbalAxis_t *ax,
    int16_t filteredError, uint8_t sequenceIsNew)
{
    int32_t dirSign, nextPos;
    int8_t  logDir;
    uint32_t pulses;

    if (!sequenceIsNew || ax->positionValid == 0U) return 2U;

    pulses = VisualGimbalXY_ErrorToPulses(filteredError, ax->deadbandPx, ax->maxSinglePulses);
    if (pulses == 0U) return 1U;

    logDir = VisualGimbalXY_ErrorToDirection(filteredError, ax->errorPositiveDir);
    if (ax->lastMoveDirection != 0 && ax->lastMoveDirection != logDir)
    {
        ax->directionChanges++;
    }

    dirSign = (logDir > 0) ? 1 : -1;
    nextPos = ax->estimatedPositionPulses + dirSign * (int32_t)pulses;
    if (nextPos > ax->softLimitPulses || nextPos < -ax->softLimitPulses)
    {
        ax->limitRejects++;
        ax->lastCommandDirection = logDir;
        ax->lastCommandPulses    = pulses;
        ax->lastCommandFreqHz    = 0U;
        ax->lastCommandAccepted  = 0U;
        return 1U;
    }

    ax->lastMoveDirection    = logDir;
    ax->lastCommandDirection = logDir;
    ax->lastCommandPulses    = pulses;
    ax->lastCommandFreqHz    = 0U;
    ax->lastCommandAccepted  = 0U;

    if (!GimbalStepper_StartMove(ax->axis, logDir, pulses, ax->stepFreqHz))
    {
        ax->startFailures++;
        return 2U;
    }

    ax->estimatedPositionPulses = nextPos;
    ax->acceptedCommands++;
    ax->lastCommandFreqHz   = ax->stepFreqHz;
    ax->lastCommandAccepted = 1U;
    return 0U;
}

void VisualGimbalXY_Init(void)
{
    uint8_t i;

    s_axisX.axis              = GIMBAL_AXIS_X;
    s_axisX.deadbandPx        = AIM_X_DEADBAND_PX;
    s_axisX.softLimitPulses   = AIM_X_SOFT_LIMIT_PULSES;
    s_axisX.stepFreqHz        = AIM_X_STEP_FREQ_HZ;
    s_axisX.maxSinglePulses   = AIM_X_MAX_SINGLE_PULSES;
    s_axisX.lockEnterPx       = AIM_X_LOCK_ENTER_PX;
    s_axisX.lockExitPx        = AIM_X_LOCK_EXIT_PX;
    s_axisX.lockConfirmFrames = AIM_X_LOCK_CONFIRM_FRAMES;
    s_axisX.errorPositiveDir  = AIM_X_ERROR_POSITIVE_DIR;

    s_axisY.axis              = GIMBAL_AXIS_Y;
    s_axisY.deadbandPx        = AIM_Y_DEADBAND_PX;
    s_axisY.softLimitPulses   = AIM_Y_SOFT_LIMIT_PULSES;
    s_axisY.stepFreqHz        = AIM_Y_STEP_FREQ_HZ;
    s_axisY.maxSinglePulses   = AIM_Y_MAX_SINGLE_PULSES;
    s_axisY.lockEnterPx       = AIM_Y_LOCK_ENTER_PX;
    s_axisY.lockExitPx        = AIM_Y_LOCK_EXIT_PX;
    s_axisY.lockConfirmFrames = AIM_Y_LOCK_CONFIRM_FRAMES;
    s_axisY.errorPositiveDir  = AIM_Y_ERROR_POSITIVE_DIR;

    for (i = 0U; i < 3U; i++) { s_axisX.errorHistory[i] = 0; s_axisY.errorHistory[i] = 0; }

    s_axisX.estimatedPositionPulses = 0; s_axisY.estimatedPositionPulses = 0;
    s_axisX.positionValid = 1U; s_axisY.positionValid = 1U;
    s_axisX.locked = 0U; s_axisY.locked = 0U;

    s_axisX.acceptedCommands = 0U; s_axisY.acceptedCommands = 0U;
    s_axisX.deadbandFrames = 0U;   s_axisY.deadbandFrames = 0U;
    s_axisX.busySkips = 0U;        s_axisY.busySkips = 0U;
    s_axisX.limitRejects = 0U;     s_axisY.limitRejects = 0U;
    s_axisX.startFailures = 0U;    s_axisY.startFailures = 0U;
    s_axisX.directionChanges = 0U; s_axisY.directionChanges = 0U;

    s_globalFaultLatched        = 0U;
    s_hasLastHandledSequence    = 0U;
    s_lastHandledSequence       = 0U;
    s_hasLastInvalidSequence    = 0U;
    s_lastInvalidSequence       = 0U;
    s_invalidFrames             = 0U;
    s_staleFrames               = 0U;
    s_skippedControlFrames      = 0U;
    s_hasLastStaleSequence      = 0U;
    s_lastStaleSequence         = 0U;
}

void VisualGimbalXY_Task10ms(void)
{
    AimObservation_t obs;
    AimLinkHealth_t  health;
    uint32_t age;
    uint16_t delta;
    uint8_t  seqNew;
    uint32_t absErr;

    if (s_globalFaultLatched) return;

    health = AimLink_GetHealth();
    if (health == AIM_LINK_FAULT) { VisualGimbalXY_LatchFault(); return; }

    if (health == AIM_LINK_NO_SIGNAL) return;

    if (!AimLink_GetLatestObservation(&obs)) return;

    age = Timer_GetMillis() - obs.rxTimeMs;
    if (age > AIM_XY_DATA_MAX_AGE_MS)
    {
        if (!s_hasLastStaleSequence || obs.sequence != s_lastStaleSequence)
        {
            s_hasLastStaleSequence = 1U; s_lastStaleSequence = obs.sequence; s_staleFrames++;
        }
        VisualGimbalXY_ResetAllFilters();
        return;
    }

    if (obs.trackingState == AIM_TRACK_FAULT) { VisualGimbalXY_LatchFault(); return; }
    if (obs.trackingState != AIM_TRACK_TRACKING)
    {
        if (!s_hasLastInvalidSequence || obs.sequence != s_lastInvalidSequence)
        { s_hasLastInvalidSequence = 1U; s_lastInvalidSequence = obs.sequence; s_invalidFrames++; }
        VisualGimbalXY_ResetAllFilters();
        return;
    }

    if (!(obs.validFlags & AIM_FLAG_RECT_VALID) ||
        !(obs.validFlags & AIM_FLAG_LASER_VALID) ||
        !(obs.validFlags & AIM_FLAG_MEASUREMENT_FRESH))
    {
        if (!s_hasLastInvalidSequence || obs.sequence != s_lastInvalidSequence)
        { s_hasLastInvalidSequence = 1U; s_lastInvalidSequence = obs.sequence; s_invalidFrames++; }
        VisualGimbalXY_ResetAllFilters();
        return;
    }

    seqNew = (!s_hasLastHandledSequence || obs.sequence != s_lastHandledSequence) ? 1U : 0U;
    if (!seqNew) return;

    if (s_hasLastHandledSequence)
    {
        delta = (uint16_t)(obs.sequence - s_lastHandledSequence);
        if (delta > 1U && delta < 0x8000U) s_skippedControlFrames += (uint32_t)(delta - 1U);
    }

    s_hasLastHandledSequence = 1U;
    s_lastHandledSequence    = obs.sequence;

    s_axisX.rawError = (int16_t)obs.rectX - (int16_t)obs.laserX;
    s_axisY.rawError = (int16_t)obs.rectY - (int16_t)obs.laserY;
    s_axisX.filteredError = VisualGimbalXY_PushFilter(&s_axisX, s_axisX.rawError);
    s_axisY.filteredError = VisualGimbalXY_PushFilter(&s_axisY, s_axisY.rawError);

    /* X-axis: lock check then move decision */
    if (!GimbalStepper_IsBusy(GIMBAL_AXIS_X))
    {
        absErr = (uint32_t)((s_axisX.filteredError >= 0) ? s_axisX.filteredError : -s_axisX.filteredError);
        if (VisualGimbalXY_ProcessAxisLock(&s_axisX, s_axisX.filteredError, absErr, 1U) == 0U)
        {
            VisualGimbalXY_ProcessAxisMove(&s_axisX, s_axisX.filteredError, 1U);
        }
    }
    else { s_axisX.busySkips++; }

    /* Y-axis: lock check then move decision */
    if (!GimbalStepper_IsBusy(GIMBAL_AXIS_Y))
    {
        absErr = (uint32_t)((s_axisY.filteredError >= 0) ? s_axisY.filteredError : -s_axisY.filteredError);
        if (VisualGimbalXY_ProcessAxisLock(&s_axisY, s_axisY.filteredError, absErr, 1U) == 0U)
        {
            VisualGimbalXY_ProcessAxisMove(&s_axisY, s_axisY.filteredError, 1U);
        }
    }
    else { s_axisY.busySkips++; }
}

void VisualGimbalXY_Stop(void)
{
    VisualGimbalXY_LatchFault();
}

void VisualGimbalXY_GetDebug(VisualGimbalXYDebug_t *outDebug)
{
    uint32_t primask;
    if (outDebug == 0) return;

    primask = __get_PRIMASK();
    __disable_irq();

    outDebug->sequence            = s_lastHandledSequence;
    outDebug->globalFaultLatched  = s_globalFaultLatched;
    outDebug->staleFrames         = s_staleFrames;
    outDebug->invalidFrames       = s_invalidFrames;
    outDebug->skippedControlFrames = s_skippedControlFrames;

    outDebug->x.rawError              = s_axisX.rawError;
    outDebug->x.filteredError         = s_axisX.filteredError;
    outDebug->x.direction             = s_axisX.lastCommandDirection;
    outDebug->x.commandPulses         = s_axisX.lastCommandPulses;
    outDebug->x.commandFreqHz         = s_axisX.lastCommandFreqHz;
    outDebug->x.commandAccepted       = s_axisX.lastCommandAccepted;
    outDebug->x.estimatedPositionPulses = s_axisX.estimatedPositionPulses;
    outDebug->x.positionValid         = s_axisX.positionValid;
    outDebug->x.locked                = s_axisX.locked;
    outDebug->x.lockConfirmCount      = s_axisX.lockConfirmCount;
    outDebug->x.lockEnterCount        = s_axisX.lockEnterCount;
    outDebug->x.busy                  = GimbalStepper_IsBusy(GIMBAL_AXIS_X);
    outDebug->x.acceptedCommands      = s_axisX.acceptedCommands;
    outDebug->x.deadbandFrames        = s_axisX.deadbandFrames;
    outDebug->x.busySkips             = s_axisX.busySkips;
    outDebug->x.limitRejects          = s_axisX.limitRejects;
    outDebug->x.startFailures         = s_axisX.startFailures;
    outDebug->x.directionChanges      = s_axisX.directionChanges;

    outDebug->y.rawError              = s_axisY.rawError;
    outDebug->y.filteredError         = s_axisY.filteredError;
    outDebug->y.direction             = s_axisY.lastCommandDirection;
    outDebug->y.commandPulses         = s_axisY.lastCommandPulses;
    outDebug->y.commandFreqHz         = s_axisY.lastCommandFreqHz;
    outDebug->y.commandAccepted       = s_axisY.lastCommandAccepted;
    outDebug->y.estimatedPositionPulses = s_axisY.estimatedPositionPulses;
    outDebug->y.positionValid         = s_axisY.positionValid;
    outDebug->y.locked                = s_axisY.locked;
    outDebug->y.lockConfirmCount      = s_axisY.lockConfirmCount;
    outDebug->y.lockEnterCount        = s_axisY.lockEnterCount;
    outDebug->y.busy                  = GimbalStepper_IsBusy(GIMBAL_AXIS_Y);
    outDebug->y.acceptedCommands      = s_axisY.acceptedCommands;
    outDebug->y.deadbandFrames        = s_axisY.deadbandFrames;
    outDebug->y.busySkips             = s_axisY.busySkips;
    outDebug->y.limitRejects          = s_axisY.limitRejects;
    outDebug->y.startFailures         = s_axisY.startFailures;
    outDebug->y.directionChanges      = s_axisY.directionChanges;

    if (primask == 0U) { __enable_irq(); }
}
