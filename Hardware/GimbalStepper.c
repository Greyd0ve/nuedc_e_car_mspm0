#include "GimbalStepper.h"
#include "app_config.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"
#include "cmsis_compiler.h"

#define GIMBAL_PERIOD_COUNTS(_freq) \
    ((uint32_t)(GIMBAL_STEP_TIMER_CLK_FREQ_HZ / (_freq)))
#define GIMBAL_DUTY50(_period)      ((_period) / 2U)
#define GIMBAL_DIR_SETUP_CYCLES     256U

typedef struct
{
    volatile uint8_t  busy;
    volatile uint8_t  done;
    uint8_t           direction;
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
} GimbalAxisCtrl_t;

static GimbalAxisCtrl_t s_axisX;
static GimbalAxisCtrl_t s_axisY;

static void GimbalStepper_InitAxis(GimbalAxisCtrl_t *axis,
    GPTIMER_Regs *timerInst,
    uint32_t iomux, uint32_t iomuxFunc,
    GPIO_Regs *stepPort, uint32_t stepPin,
    uint32_t dirIomux, GPIO_Regs *dirPort, uint32_t dirPin)
{
    uint32_t i;
    volatile uint8_t *p = (volatile uint8_t *)axis;

    for (i = 0U; i < (uint32_t)sizeof(GimbalAxisCtrl_t); i++)
    {
        p[i] = 0U;
    }

    axis->timerInst   = timerInst;
    axis->ccIndex     = DL_TIMER_CC_0_INDEX;
    axis->iomux       = iomux;
    axis->iomuxFunc   = iomuxFunc;
    axis->dirPort     = dirPort;
    axis->dirPin      = dirPin;
    axis->dirIomux    = dirIomux;
    axis->stepPort    = stepPort;
    axis->stepPin     = stepPin;

    DL_GPIO_initDigitalOutput(dirIomux);
    DL_GPIO_clearPins(dirPort, dirPin);
}

static void GimbalStepper_ConfigTimer(GPTIMER_Regs *timerInst,
    uint32_t periodCounts, uint8_t ccIndex)
{
    DL_TimerA_ClockConfig clkCfg = {
        .clockSel    = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale    = 31U
    };
    DL_TimerA_PWMConfig pwmCfg = {
        .pwmMode            = DL_TIMER_PWM_MODE_EDGE_ALIGN,
        .period             = periodCounts,
        .isTimerWithFourCC  = false,
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
    DL_TimerA_clearInterruptStatus(timerInst, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableInterrupt(timerInst, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(timerInst);
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

void GimbalStepper_Init(void)
{
#if GIMBAL_USE_ENABLE_GPIO
    DL_GPIO_initDigitalOutput(IOMUX_PINCM44);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM56);
    DL_GPIO_clearPins(GIMBAL_X_EN_PORT, GIMBAL_X_EN_PIN);
    DL_GPIO_clearPins(GIMBAL_Y_EN_PORT, GIMBAL_Y_EN_PIN);
#endif

    DL_TimerA_reset(TIMA1);
    DL_TimerA_enablePower(TIMA1);
    delay_cycles(POWER_STARTUP_DELAY);

    GimbalStepper_InitAxis(&s_axisX, TIMA0,
        GIMBAL_X_STEP_IOMUX, GIMBAL_X_STEP_IOMUX_FUNC,
        GIMBAL_X_STEP_PORT, GIMBAL_X_STEP_PIN,
        GIMBAL_X_DIR_IOMUX, GIMBAL_X_DIR_PORT, GIMBAL_X_DIR_PIN);

    GimbalStepper_InitAxis(&s_axisY, TIMA1,
        GIMBAL_Y_STEP_IOMUX, GIMBAL_Y_STEP_IOMUX_FUNC,
        GIMBAL_Y_STEP_PORT, GIMBAL_Y_STEP_PIN,
        GIMBAL_Y_DIR_IOMUX, GIMBAL_Y_DIR_PORT, GIMBAL_Y_DIR_PIN);

    GimbalStepper_ForceStepLow(&s_axisX);
    GimbalStepper_ForceStepLow(&s_axisY);
}

uint8_t GimbalStepper_StartMove(GimbalAxis_t axis,
    int8_t direction, uint32_t pulseCount, uint32_t frequencyHz)
{
    GimbalAxisCtrl_t *pAxis;
    uint32_t period;
    uint32_t primask;

    if (axis == GIMBAL_AXIS_X)
    {
        pAxis = &s_axisX;
    }
    else
    {
        pAxis = &s_axisY;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (pAxis->busy != 0U)
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    if (pulseCount == 0U || frequencyHz == 0U)
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    period = GIMBAL_PERIOD_COUNTS(frequencyHz);
    if (period < 2U)
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    pAxis->busy            = 1U;
    pAxis->done            = 0U;
    pAxis->targetPulses    = pulseCount;
    pAxis->emittedPulses   = 0U;
    pAxis->remainingPulses = pulseCount;
    pAxis->periodCounts    = period;
    pAxis->direction       = (direction >= 0) ? 1U : 0U;

    if (pAxis->direction != 0U)
    {
        DL_GPIO_setPins(pAxis->dirPort, pAxis->dirPin);
    }
    else
    {
        DL_GPIO_clearPins(pAxis->dirPort, pAxis->dirPin);
    }

    delay_cycles(GIMBAL_DIR_SETUP_CYCLES);

    GimbalStepper_ConfigTimer(pAxis->timerInst, period, pAxis->ccIndex);
    DL_TimerA_setCaptureCompareValue(pAxis->timerInst,
        GIMBAL_DUTY50(period), pAxis->ccIndex);

    GimbalStepper_ForceStepPeripheral(pAxis);

    DL_TimerA_clearInterruptStatus(pAxis->timerInst, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(
        (axis == GIMBAL_AXIS_X) ? TIMA0_INT_IRQn : TIMA1_INT_IRQn);
    NVIC_EnableIRQ(
        (axis == GIMBAL_AXIS_X) ? TIMA0_INT_IRQn : TIMA1_INT_IRQn);
    DL_TimerA_startCounter(pAxis->timerInst);

    if (primask == 0U) { __enable_irq(); }

    return 1U;
}

void GimbalStepper_Stop(GimbalAxis_t axis)
{
    GimbalAxisCtrl_t *pAxis;
    GPTIMER_Regs *timer;
    IRQn_Type irq;

    if (axis == GIMBAL_AXIS_X)
    {
        pAxis = &s_axisX;
        irq   = TIMA0_INT_IRQn;
    }
    else
    {
        pAxis = &s_axisY;
        irq   = TIMA1_INT_IRQn;
    }

    timer = pAxis->timerInst;

    NVIC_DisableIRQ(irq);
    DL_TimerA_stopCounter(timer);
    DL_TimerA_clearInterruptStatus(timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(irq);

    GimbalStepper_ForceStepLow(pAxis);

    pAxis->busy            = 0U;
    pAxis->done            = 0U;
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
    if (axis == GIMBAL_AXIS_X) { return s_axisX.busy; }
    return s_axisY.busy;
}

uint32_t GimbalStepper_GetRemainingPulses(GimbalAxis_t axis)
{
    if (axis == GIMBAL_AXIS_X) { return s_axisX.remainingPulses; }
    return s_axisY.remainingPulses;
}

uint8_t GimbalStepper_IsDone(GimbalAxis_t axis)
{
    uint8_t val;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (axis == GIMBAL_AXIS_X) { val = s_axisX.done; }
    else                       { val = s_axisY.done; }
    if (primask == 0U) { __enable_irq(); }
    return val;
}

void GimbalStepper_ClearDone(GimbalAxis_t axis)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (axis == GIMBAL_AXIS_X) { s_axisX.done = 0U; }
    else                       { s_axisY.done = 0U; }
    if (primask == 0U) { __enable_irq(); }
}

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
                DL_TimerA_stopCounter(pAxis->timerInst);
                DL_TimerA_clearInterruptStatus(pAxis->timerInst,
                    DL_TIMER_INTERRUPT_ZERO_EVENT);
                NVIC_DisableIRQ(irq);

                GimbalStepper_ForceStepLow(pAxis);

                pAxis->busy  = 0U;
                pAxis->done  = 1U;
                pAxis->remainingPulses = 0U;
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
