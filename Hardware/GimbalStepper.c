#include "GimbalStepper.h"
#include "app_config.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"
#include "cmsis_compiler.h"

/*
 * Edge-aligned PWM, DL_TIMER_CC_OCTL_INIT_VAL_LOW:
 *   Zero event  → counter reload, output goes HIGH  → physical STEP rising edge
 *   Compare CC0 → output goes LOW                   → physical STEP falling edge
 *
 * Pulse counting:
 *   Each zero event = 1 rising edge emitted.
 *   When emitted reaches target, disable zero interrupt and arm CC0 interrupt.
 *   CC0 interrupt fires → falling edge of last pulse complete → finalize stop.
 *
 * Result: exactly target rising edges with full high-width on last pulse.
 */
#define GIMBAL_PERIOD_COUNTS(_freq) \
    ((uint32_t)(GIMBAL_STEP_TIMER_CLK_FREQ_HZ / (_freq)))
#define GIMBAL_DUTY50(_period)      ((_period) / 2U)
#define GIMBAL_DIR_SETUP_CYCLES     256U

typedef struct
{
    volatile uint8_t  busy;
    volatile uint8_t  done;
    volatile uint8_t  stopPending;
    int8_t            logicalDirection;
    uint8_t           positiveDirLevel;
    uint8_t           dirOutputLevel;
    uint8_t           isTimerWithFourCC;
    uint32_t          targetPulses;
    uint32_t          emittedPulses;
    uint32_t          remainingPulses;
    uint32_t          periodCounts;
    GPTIMER_Regs     *timerInst;
    uint8_t           ccIndex;
    uint32_t          iomux;
    uint32_t          iomuxFunc;
    GPIO_Regs        *dirPort;
    uint32_t          dirPin;
    uint32_t          dirIomux;
    GPIO_Regs        *stepPort;
    uint32_t          stepPin;
#if GIMBAL_USE_ENABLE_GPIO
    GPIO_Regs        *enPort;
    uint32_t          enPin;
#endif
} GimbalAxisCtrl_t;

static GimbalAxisCtrl_t s_axisX;
static GimbalAxisCtrl_t s_axisY;

static GimbalAxisCtrl_t *GimbalStepper_GetAxis(GimbalAxis_t axis)
{
    if (axis == GIMBAL_AXIS_X) { return &s_axisX; }
    if (axis == GIMBAL_AXIS_Y) { return &s_axisY; }
    return (GimbalAxisCtrl_t *)0;
}

static IRQn_Type GimbalStepper_GetIRQn(GimbalAxis_t axis)
{
    if (axis == GIMBAL_AXIS_X) { return TIMA0_INT_IRQn; }
    return TIMA1_INT_IRQn;
}

#if GIMBAL_USE_ENABLE_GPIO
static void GimbalStepper_WriteEN(GimbalAxisCtrl_t *axis, uint8_t enable)
{
    if (enable != 0U)
    {
        if (GIMBAL_EN_ACTIVE_LEVEL != 0U)
        {
            DL_GPIO_setPins(axis->enPort, axis->enPin);
        }
        else
        {
            DL_GPIO_clearPins(axis->enPort, axis->enPin);
        }
    }
    else
    {
        if (GIMBAL_EN_ACTIVE_LEVEL != 0U)
        {
            DL_GPIO_clearPins(axis->enPort, axis->enPin);
        }
        else
        {
            DL_GPIO_setPins(axis->enPort, axis->enPin);
        }
    }
}
#else
#define GimbalStepper_WriteEN(axis, enable) ((void)(axis), (void)(enable))
#endif

static void GimbalStepper_WriteDIR(GimbalAxisCtrl_t *axis)
{
    if (axis->dirOutputLevel != 0U)
    {
        DL_GPIO_setPins(axis->dirPort, axis->dirPin);
    }
    else
    {
        DL_GPIO_clearPins(axis->dirPort, axis->dirPin);
    }
}

static void GimbalStepper_ComputeDirection(GimbalAxisCtrl_t *axis,
    int8_t logicalDirection)
{
    axis->logicalDirection = logicalDirection;
    if (logicalDirection > 0)
    {
        axis->dirOutputLevel = axis->positiveDirLevel;
    }
    else
    {
        axis->dirOutputLevel =
            (axis->positiveDirLevel != 0U) ? 0U : 1U;
    }
}

static void GimbalStepper_StopHardware(GimbalAxisCtrl_t *axis, IRQn_Type irq)
{
    NVIC_DisableIRQ(irq);
    DL_TimerA_stopCounter(axis->timerInst);
    DL_TimerA_clearInterruptStatus(axis->timerInst,
        DL_TIMER_INTERRUPT_ZERO_EVENT | DL_TIMER_INTERRUPT_CC0_UP_EVENT);
    NVIC_ClearPendingIRQ(irq);
}

static void GimbalStepper_ForceStepLow(GimbalAxisCtrl_t *axis)
{
    DL_GPIO_initDigitalOutput(axis->iomux);
    DL_GPIO_clearPins(axis->stepPort, axis->stepPin);
}

static void GimbalStepper_ForceStepPeripheral(GimbalAxisCtrl_t *axis)
{
    DL_GPIO_initPeripheralOutputFunction(axis->iomux, axis->iomuxFunc);
    DL_GPIO_enableOutput(axis->stepPort, axis->stepPin);
}

static void GimbalStepper_InitAxis(GimbalAxisCtrl_t *axis,
    GPTIMER_Regs *timerInst, uint8_t isTimerWithFourCC,
    uint32_t iomux, uint32_t iomuxFunc,
    GPIO_Regs *stepPort, uint32_t stepPin,
    uint32_t dirIomux, GPIO_Regs *dirPort, uint32_t dirPin,
    uint8_t positiveDirLevel
#if GIMBAL_USE_ENABLE_GPIO
    , GPIO_Regs *enPort, uint32_t enPin
#endif
    )
{
    uint32_t i;
    volatile uint8_t *p = (volatile uint8_t *)axis;

    for (i = 0U; i < (uint32_t)sizeof(GimbalAxisCtrl_t); i++)
    {
        p[i] = 0U;
    }

    axis->timerInst         = timerInst;
    axis->isTimerWithFourCC = isTimerWithFourCC;
    axis->ccIndex           = DL_TIMER_CC_0_INDEX;
    axis->iomux             = iomux;
    axis->iomuxFunc         = iomuxFunc;
    axis->dirPort           = dirPort;
    axis->dirPin            = dirPin;
    axis->dirIomux          = dirIomux;
    axis->stepPort          = stepPort;
    axis->stepPin           = stepPin;
    axis->positiveDirLevel  = positiveDirLevel;
#if GIMBAL_USE_ENABLE_GPIO
    axis->enPort = enPort;
    axis->enPin  = enPin;
#endif

    DL_GPIO_initDigitalOutput(dirIomux);
    DL_GPIO_clearPins(dirPort, dirPin);

#if GIMBAL_USE_ENABLE_GPIO
    DL_GPIO_initDigitalOutput(
        (axis == &s_axisX) ? GPIO_GIMBAL_X_EN_IOMUX : GPIO_GIMBAL_Y_EN_IOMUX);
    GimbalStepper_WriteEN(axis, 0U);
#endif
}

static void GimbalStepper_ConfigTimer(GPTIMER_Regs *timerInst,
    uint32_t periodCounts, uint8_t isTimerWithFourCC, uint8_t ccIndex)
{
    DL_TimerA_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = GIMBAL_STEP_TIMER_PRESCALE
    };
    DL_TimerA_PWMConfig pwmCfg = {
        .pwmMode            = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period             = periodCounts,
        .isTimerWithFourCC  = (isTimerWithFourCC != 0U),
        .startTimer         = DL_TIMER_STOP,
    };

    DL_TimerA_stopCounter(timerInst);
    DL_TimerA_setClockConfig(timerInst, &clkCfg);
    DL_TimerA_initPWMMode(timerInst, &pwmCfg);
    DL_TimerA_setCaptureCompareOutCtl(timerInst,
        DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
        DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        ccIndex);
    DL_TimerA_setCaptCompUpdateMethod(timerInst,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, ccIndex);
    DL_TimerA_setCaptureCompareValue(timerInst, periodCounts, ccIndex);
    DL_TimerA_setCCPDirection(timerInst, DL_TIMER_CC0_OUTPUT);
    DL_TimerA_clearInterruptStatus(timerInst,
        DL_TIMER_INTERRUPT_ZERO_EVENT | DL_TIMER_INTERRUPT_CC0_UP_EVENT);
    DL_TimerA_enableInterrupt(timerInst, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(timerInst);
}

void GimbalStepper_Init(void)
{
    DL_TimerA_reset(TIMA1);
    DL_TimerA_enablePower(TIMA1);
    delay_cycles(POWER_STARTUP_DELAY);

    GimbalStepper_InitAxis(&s_axisX, TIMA0, 1U,
        GIMBAL_X_STEP_IOMUX, GIMBAL_X_STEP_IOMUX_FUNC,
        GIMBAL_X_STEP_PORT, GIMBAL_X_STEP_PIN,
        GIMBAL_X_DIR_IOMUX, GIMBAL_X_DIR_PORT, GIMBAL_X_DIR_PIN,
        GIMBAL_X_POSITIVE_DIR_LEVEL
#if GIMBAL_USE_ENABLE_GPIO
        , GIMBAL_X_EN_PORT, GIMBAL_X_EN_PIN
#endif
        );

    GimbalStepper_InitAxis(&s_axisY, TIMA1, 0U,
        GIMBAL_Y_STEP_IOMUX, GIMBAL_Y_STEP_IOMUX_FUNC,
        GIMBAL_Y_STEP_PORT, GIMBAL_Y_STEP_PIN,
        GIMBAL_Y_DIR_IOMUX, GIMBAL_Y_DIR_PORT, GIMBAL_Y_DIR_PIN,
        GIMBAL_Y_POSITIVE_DIR_LEVEL
#if GIMBAL_USE_ENABLE_GPIO
        , GIMBAL_Y_EN_PORT, GIMBAL_Y_EN_PIN
#endif
        );

    GimbalStepper_ForceStepLow(&s_axisX);
    GimbalStepper_ForceStepLow(&s_axisY);
}

uint8_t GimbalStepper_StartMove(GimbalAxis_t axis,
    int8_t logicalDirection, uint32_t pulseCount, uint32_t frequencyHz)
{
    GimbalAxisCtrl_t *pAxis;
    uint32_t period;
    uint32_t primask;

    pAxis = GimbalStepper_GetAxis(axis);
    if (pAxis == (GimbalAxisCtrl_t *)0) { return 0U; }

    if (logicalDirection != GIMBAL_DIR_POSITIVE &&
        logicalDirection != GIMBAL_DIR_NEGATIVE)
    {
        return 0U;
    }

    if (pulseCount == 0U || frequencyHz == 0U) { return 0U; }

    period = GIMBAL_PERIOD_COUNTS(frequencyHz);
    if (period < 2U) { return 0U; }

    primask = __get_PRIMASK();
    __disable_irq();

    if (pAxis->busy != 0U)
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    pAxis->busy            = 1U;
    pAxis->done            = 0U;
    pAxis->stopPending     = 0U;
    pAxis->targetPulses    = pulseCount;
    pAxis->emittedPulses   = 0U;
    pAxis->remainingPulses = pulseCount;
    pAxis->periodCounts    = period;

    GimbalStepper_ComputeDirection(pAxis, logicalDirection);
    GimbalStepper_WriteDIR(pAxis);
    delay_cycles(GIMBAL_DIR_SETUP_CYCLES);

    GimbalStepper_WriteEN(pAxis, 1U);

    GimbalStepper_ConfigTimer(pAxis->timerInst, period,
        pAxis->isTimerWithFourCC, pAxis->ccIndex);
    DL_TimerA_setCaptureCompareValue(pAxis->timerInst,
        GIMBAL_DUTY50(period), pAxis->ccIndex);

    GimbalStepper_ForceStepPeripheral(pAxis);

    DL_TimerA_clearInterruptStatus(pAxis->timerInst,
        DL_TIMER_INTERRUPT_ZERO_EVENT | DL_TIMER_INTERRUPT_CC0_UP_EVENT);
    NVIC_ClearPendingIRQ(GimbalStepper_GetIRQn(axis));
    NVIC_EnableIRQ(GimbalStepper_GetIRQn(axis));
    DL_TimerA_startCounter(pAxis->timerInst);

    if (primask == 0U) { __enable_irq(); }

    return 1U;
}

void GimbalStepper_Stop(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis;
    IRQn_Type irq;

    pAxis = GimbalStepper_GetAxis(axis);
    if (pAxis == (GimbalAxisCtrl_t *)0) { return; }

    irq = GimbalStepper_GetIRQn(axis);

    GimbalStepper_StopHardware(pAxis, irq);

    GimbalStepper_WriteEN(pAxis, 0U);
    GimbalStepper_ForceStepLow(pAxis);

    pAxis->busy            = 0U;
    pAxis->done            = 0U;
    pAxis->stopPending     = 0U;
    pAxis->emittedPulses   = 0U;
    pAxis->remainingPulses = 0U;
    pAxis->targetPulses    = 0U;
}

void GimbalStepper_StopAll(void)
{
    GimbalStepper_Stop(GIMBAL_AXIS_X);
    GimbalStepper_Stop(GIMBAL_AXIS_Y);
}

uint8_t GimbalStepper_IsBusy(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis = GimbalStepper_GetAxis(axis);
    if (pAxis == (GimbalAxisCtrl_t *)0) { return 0U; }
    return pAxis->busy;
}

uint32_t GimbalStepper_GetRemainingPulses(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis = GimbalStepper_GetAxis(axis);
    if (pAxis == (GimbalAxisCtrl_t *)0) { return 0U; }
    return pAxis->remainingPulses;
}

uint8_t GimbalStepper_IsDone(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis = GimbalStepper_GetAxis(axis);
    uint8_t val;
    uint32_t primask;

    if (pAxis == (GimbalAxisCtrl_t *)0) { return 0U; }

    primask = __get_PRIMASK();
    __disable_irq();
    val = pAxis->done;
    if (primask == 0U) { __enable_irq(); }
    return val;
}

void GimbalStepper_ClearDone(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis = GimbalStepper_GetAxis(axis);
    uint32_t primask;

    if (pAxis == (GimbalAxisCtrl_t *)0) { return; }

    primask = __get_PRIMASK();
    __disable_irq();
    pAxis->done = 0U;
    if (primask == 0U) { __enable_irq(); }
}

/*
 * ISR: Two-stage stop for exact pulse count.
 * Zero event  = rising  edge of a new pulse.
 * CC0 compare = falling edge completing the current pulse.
 *
 * Stage 1 (zero event): emitted reaches target → disable zero int, arm CC0 int.
 * Stage 2 (CC0 compare): last pulse's falling edge done → stop hardware, set done.
 */
static void GimbalStepper_ISR(GimbalAxisCtrl_t *pAxis, IRQn_Type irq)
{
    switch (DL_TimerA_getPendingInterrupt(pAxis->timerInst))
    {
        case DL_TIMER_IIDX_ZERO:
            pAxis->emittedPulses++;
            if (pAxis->emittedPulses <= pAxis->targetPulses)
            {
                pAxis->remainingPulses =
                    pAxis->targetPulses - pAxis->emittedPulses;
            }

            if (pAxis->emittedPulses >= pAxis->targetPulses)
            {
                DL_TimerA_disableInterrupt(pAxis->timerInst,
                    DL_TIMER_INTERRUPT_ZERO_EVENT);
                DL_TimerA_clearInterruptStatus(pAxis->timerInst,
                    DL_TIMER_INTERRUPT_ZERO_EVENT);
                DL_TimerA_enableInterrupt(pAxis->timerInst,
                    DL_TIMER_INTERRUPT_CC0_UP_EVENT);
                pAxis->stopPending = 1U;
            }
            break;

        case DL_TIMER_IIDX_CC0_UP:
            if (pAxis->stopPending != 0U)
            {
                GimbalStepper_StopHardware(pAxis, irq);

                GimbalStepper_WriteEN(pAxis, 0U);
                GimbalStepper_ForceStepLow(pAxis);

                pAxis->busy            = 0U;
                pAxis->stopPending     = 0U;
                pAxis->done            = 1U;
                pAxis->remainingPulses = 0U;
                pAxis->emittedPulses   = 0U;
            }
            break;

        default:
            break;
    }
}

void TIMA0_IRQHandler(void)
{
    GimbalStepper_ISR(&s_axisX, TIMA0_INT_IRQn);
}

void TIMA1_IRQHandler(void)
{
    GimbalStepper_ISR(&s_axisY, TIMA1_INT_IRQn);
}
