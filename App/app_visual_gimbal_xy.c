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
static uint8_t  s_hasLastStaleSequence;
static uint16_t s_lastStaleSequence;
static uint32_t s_lastObservationAgeMs;

static void VisualGimbalXY_ClearAxisCommand(VisualGimbalAxis_t *ax)
{
    ax->lastCommandDirection = 0;
    ax->lastCommandPulses    = 0U;
    ax->lastCommandFreqHz    = 0U;
    ax->lastCommandAccepted  = 0U;
}

static void VisualGimbalXY_InvalidateAxisError(VisualGimbalAxis_t *ax)
{
    ax->rawError    = 0;
    ax->filteredError = 0;
}

static void VisualGimbalXY_ResetAxisFilterAndLock(VisualGimbalAxis_t *ax)
{
    uint8_t i;
    for (i = 0U; i < 3U; i++) { ax->errorHistory[i] = 0; }
    ax->errorHistoryCount = 0U;
    ax->errorHistoryIndex = 0U;
    ax->locked            = 0U;
    ax->lockConfirmCount  = 0U;
}

static void VisualGimbalXY_ResetAllState(void)
{
    VisualGimbalXY_ResetAxisFilterAndLock(&s_axisX);
    VisualGimbalXY_ResetAxisFilterAndLock(&s_axisY);
    VisualGimbalXY_ClearAxisCommand(&s_axisX);
    VisualGimbalXY_ClearAxisCommand(&s_axisY);
    VisualGimbalXY_InvalidateAxisError(&s_axisX);
    VisualGimbalXY_InvalidateAxisError(&s_axisY);
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
    VisualGimbalXY_ResetAllState();
}

static uint32_t VisualGimbalXY_MinU32(uint32_t a, uint32_t b) { return (a < b) ? a : b; }

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
        return (int16_t)(((int32_t)ax->errorHistory[0] + (int32_t)ax->errorHistory[1]) / 2);
    return VisualGimbalXY_MedianOfThree(ax->errorHistory[0], ax->errorHistory[1], ax->errorHistory[2]);
}

static uint32_t VisualGimbalXY_ErrorToPulses(int16_t error, uint32_t deadband, uint32_t maxPulses)
{
    uint32_t absErr = (uint32_t)((error >= 0) ? error : -error);
    if (absErr <= deadband)  return 0U;
    if (absErr <= 12U)       return VisualGimbalXY_MinU32(1U, maxPulses);
    if (absErr <= 30U)       return VisualGimbalXY_MinU32(2U, maxPulses);
    if (absErr <= 60U)       return VisualGimbalXY_MinU32(4U, maxPulses);
    if (absErr <= 120U)      return VisualGimbalXY_MinU32(6U, maxPulses);
    return maxPulses;
}

static int8_t VisualGimbalXY_ErrorToDirection(int16_t error, int8_t posDir)
{
    if (error > 0) return (posDir > 0) ? GIMBAL_DIR_POSITIVE : GIMBAL_DIR_NEGATIVE;
    return (posDir > 0) ? GIMBAL_DIR_NEGATIVE : GIMBAL_DIR_POSITIVE;
}

/*
 * Returns: 0 = proceed to move, 1 = no move (locked or confirming),
 *          2 = axis position invalid (no move).
 */
static uint8_t VisualGimbalXY_ProcessAxisLock(VisualGimbalAxis_t *ax, uint32_t absErr)
{
    if (ax->positionValid == 0U) return 2U;

    if (ax->locked)
    {
        if (absErr <= ax->lockExitPx) return 1U;
        ax->locked           = 0U;
        ax->lockConfirmCount = 0U;
        return 0U;
    }

    if (absErr <= ax->lockEnterPx)
    {
        ax->lockConfirmCount++;
        if (ax->lockConfirmCount >= ax->lockConfirmFrames)
        {
            ax->locked         = 1U;
            ax->lockEnterCount++;
        }
        return 1U;
    }
    ax->lockConfirmCount = 0U;
    return 0U;
}

/*
 * Returns: 0 = move started, 1 = deadband/limit/skip, 2 = start failed.
 * Only updates directionChanges/estimatedPosition on success.
 */
static uint8_t VisualGimbalXY_ProcessAxisMove(VisualGimbalAxis_t *ax, int16_t filteredError)
{
    int32_t dirSign, nextPos;
    int8_t  logDir;
    uint32_t pulses;

    pulses = VisualGimbalXY_ErrorToPulses(filteredError, ax->deadbandPx, ax->maxSinglePulses);
    if (pulses == 0U) { ax->deadbandFrames++; return 1U; }

    logDir = VisualGimbalXY_ErrorToDirection(filteredError, ax->errorPositiveDir);

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

    ax->lastCommandDirection = logDir;
    ax->lastCommandPulses    = pulses;
    ax->lastCommandFreqHz    = 0U;
    ax->lastCommandAccepted  = 0U;

    if (!GimbalStepper_StartMove(ax->axis, logDir, pulses, ax->stepFreqHz))
    {
        ax->startFailures++;
        return 2U;
    }

    if (ax->lastMoveDirection != 0 && ax->lastMoveDirection != logDir)
        ax->directionChanges++;
    ax->lastMoveDirection = logDir;

    ax->estimatedPositionPulses = nextPos;
    ax->acceptedCommands++;
    ax->lastCommandFreqHz   = ax->stepFreqHz;
    ax->lastCommandAccepted = 1U;
    return 0U;
}

static void VisualGimbalXY_InitAxis(VisualGimbalAxis_t *ax,
    GimbalAxis_t axis, uint32_t db, int32_t sl, uint32_t freq, uint32_t mp,
    uint32_t le, uint32_t lx, uint32_t lc, int8_t ed)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)ax;
    for (i = 0U; i < (uint32_t)sizeof(*ax); i++) { p[i] = 0U; }
    ax->axis              = axis;
    ax->deadbandPx        = db;
    ax->softLimitPulses   = sl;
    ax->stepFreqHz        = freq;
    ax->maxSinglePulses   = mp;
    ax->lockEnterPx       = le;
    ax->lockExitPx        = lx;
    ax->lockConfirmFrames = lc;
    ax->errorPositiveDir  = ed;
    ax->positionValid     = 1U;
}

void VisualGimbalXY_Init(void)
{
    VisualGimbalXY_InitAxis(&s_axisX, GIMBAL_AXIS_X,
        AIM_X_DEADBAND_PX, AIM_X_SOFT_LIMIT_PULSES,
        AIM_X_STEP_FREQ_HZ, AIM_X_MAX_SINGLE_PULSES,
        AIM_X_LOCK_ENTER_PX, AIM_X_LOCK_EXIT_PX,
        AIM_X_LOCK_CONFIRM_FRAMES, AIM_X_ERROR_POSITIVE_DIR);

    VisualGimbalXY_InitAxis(&s_axisY, GIMBAL_AXIS_Y,
        AIM_Y_DEADBAND_PX, AIM_Y_SOFT_LIMIT_PULSES,
        AIM_Y_STEP_FREQ_HZ, AIM_Y_MAX_SINGLE_PULSES,
        AIM_Y_LOCK_ENTER_PX, AIM_Y_LOCK_EXIT_PX,
        AIM_Y_LOCK_CONFIRM_FRAMES, AIM_Y_ERROR_POSITIVE_DIR);

    s_globalFaultLatched       = 0U;
    s_hasLastHandledSequence   = 0U;
    s_lastHandledSequence      = 0U;
    s_hasLastInvalidSequence   = 0U;
    s_lastInvalidSequence      = 0U;
    s_invalidFrames            = 0U;
    s_staleFrames              = 0U;
    s_skippedControlFrames     = 0U;
    s_hasLastStaleSequence     = 0U;
    s_lastStaleSequence        = 0U;
    s_lastObservationAgeMs     = 0xFFFFFFFFUL;
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

    if (health == AIM_LINK_NO_SIGNAL)
    {
        s_lastObservationAgeMs = 0xFFFFFFFFUL;
        VisualGimbalXY_ResetAllState();
        return;
    }

    if (!AimLink_GetLatestObservation(&obs)) return;

    age = Timer_GetMillis() - obs.rxTimeMs;
    s_lastObservationAgeMs = age;

    if (age > AIM_XY_DATA_MAX_AGE_MS)
    {
        if (!s_hasLastStaleSequence || obs.sequence != s_lastStaleSequence)
        { s_hasLastStaleSequence = 1U; s_lastStaleSequence = obs.sequence; s_staleFrames++; }
        VisualGimbalXY_ResetAllState();
        return;
    }

    if (obs.trackingState == AIM_TRACK_FAULT) { VisualGimbalXY_LatchFault(); return; }
    if (obs.trackingState != AIM_TRACK_TRACKING)
    {
        if (!s_hasLastInvalidSequence || obs.sequence != s_lastInvalidSequence)
        { s_hasLastInvalidSequence = 1U; s_lastInvalidSequence = obs.sequence; s_invalidFrames++; }
        VisualGimbalXY_ResetAllState();
        return;
    }

    if (!(obs.validFlags & AIM_FLAG_RECT_VALID) ||
        !(obs.validFlags & AIM_FLAG_LASER_VALID) ||
        !(obs.validFlags & AIM_FLAG_MEASUREMENT_FRESH))
    {
        if (!s_hasLastInvalidSequence || obs.sequence != s_lastInvalidSequence)
        { s_hasLastInvalidSequence = 1U; s_lastInvalidSequence = obs.sequence; s_invalidFrames++; }
        VisualGimbalXY_ResetAllState();
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

    VisualGimbalXY_ClearAxisCommand(&s_axisX);
    VisualGimbalXY_ClearAxisCommand(&s_axisY);

    s_axisX.rawError = (int16_t)obs.rectX - (int16_t)obs.laserX;
    s_axisY.rawError = (int16_t)obs.rectY - (int16_t)obs.laserY;
    s_axisX.filteredError = VisualGimbalXY_PushFilter(&s_axisX, s_axisX.rawError);
    s_axisY.filteredError = VisualGimbalXY_PushFilter(&s_axisY, s_axisY.rawError);

    /* X-axis */
    if (!GimbalStepper_IsBusy(GIMBAL_AXIS_X))
    {
        absErr = (uint32_t)((s_axisX.filteredError >= 0) ? s_axisX.filteredError : -s_axisX.filteredError);
        if (absErr <= s_axisX.deadbandPx) { s_axisX.deadbandFrames++; }
        if (VisualGimbalXY_ProcessAxisLock(&s_axisX, absErr) == 0U)
        {
            VisualGimbalXY_ProcessAxisMove(&s_axisX, s_axisX.filteredError);
        }
    }
    else { s_axisX.busySkips++; }

    /* Y-axis */
    if (!GimbalStepper_IsBusy(GIMBAL_AXIS_Y))
    {
        absErr = (uint32_t)((s_axisY.filteredError >= 0) ? s_axisY.filteredError : -s_axisY.filteredError);
        if (absErr <= s_axisY.deadbandPx) { s_axisY.deadbandFrames++; }
        if (VisualGimbalXY_ProcessAxisLock(&s_axisY, absErr) == 0U)
        {
            VisualGimbalXY_ProcessAxisMove(&s_axisY, s_axisY.filteredError);
        }
    }
    else { s_axisY.busySkips++; }
}

void VisualGimbalXY_Stop(void) { VisualGimbalXY_LatchFault(); }

void VisualGimbalXY_GetDebug(VisualGimbalXYDebug_t *outDebug)
{
    uint32_t primask;
    if (outDebug == 0) return;

    primask = __get_PRIMASK();
    __disable_irq();

    outDebug->sequence             = s_lastHandledSequence;
    outDebug->ageMs                = s_lastObservationAgeMs;
    outDebug->globalFaultLatched   = s_globalFaultLatched;
    outDebug->xyLocked             = (uint8_t)((s_axisX.locked && s_axisY.locked) ? 1U : 0U);
    outDebug->staleFrames          = s_staleFrames;
    outDebug->invalidFrames        = s_invalidFrames;
    outDebug->skippedControlFrames = s_skippedControlFrames;

    outDebug->x = (VisualAxisDebug_t){
        .rawError              = s_axisX.rawError,
        .filteredError         = s_axisX.filteredError,
        .direction             = s_axisX.lastCommandDirection,
        .commandPulses         = s_axisX.lastCommandPulses,
        .commandFreqHz         = s_axisX.lastCommandFreqHz,
        .commandAccepted       = s_axisX.lastCommandAccepted,
        .estimatedPositionPulses = s_axisX.estimatedPositionPulses,
        .positionValid         = s_axisX.positionValid,
        .locked                = s_axisX.locked,
        .lockConfirmCount      = s_axisX.lockConfirmCount,
        .lockEnterCount        = s_axisX.lockEnterCount,
        .busy                  = GimbalStepper_IsBusy(GIMBAL_AXIS_X),
        .acceptedCommands      = s_axisX.acceptedCommands,
        .deadbandFrames        = s_axisX.deadbandFrames,
        .busySkips             = s_axisX.busySkips,
        .limitRejects          = s_axisX.limitRejects,
        .startFailures         = s_axisX.startFailures,
        .directionChanges      = s_axisX.directionChanges,
    };

    outDebug->y = (VisualAxisDebug_t){
        .rawError              = s_axisY.rawError,
        .filteredError         = s_axisY.filteredError,
        .direction             = s_axisY.lastCommandDirection,
        .commandPulses         = s_axisY.lastCommandPulses,
        .commandFreqHz         = s_axisY.lastCommandFreqHz,
        .commandAccepted       = s_axisY.lastCommandAccepted,
        .estimatedPositionPulses = s_axisY.estimatedPositionPulses,
        .positionValid         = s_axisY.positionValid,
        .locked                = s_axisY.locked,
        .lockConfirmCount      = s_axisY.lockConfirmCount,
        .lockEnterCount        = s_axisY.lockEnterCount,
        .busy                  = GimbalStepper_IsBusy(GIMBAL_AXIS_Y),
        .acceptedCommands      = s_axisY.acceptedCommands,
        .deadbandFrames        = s_axisY.deadbandFrames,
        .busySkips             = s_axisY.busySkips,
        .limitRejects          = s_axisY.limitRejects,
        .startFailures         = s_axisY.startFailures,
        .directionChanges      = s_axisY.directionChanges,
    };

    if (primask == 0U) { __enable_irq(); }
}
