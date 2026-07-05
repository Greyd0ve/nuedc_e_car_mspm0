#include "app_line.h"
#include "Grayscale.h"
#include <stdint.h>

#ifndef GRAYSCALE_CHANNELS
#define GRAYSCALE_CHANNELS 8U
#endif

#define APP_LINE_CONTROL_PERIOD_MS 10U

extern volatile float g_lineBlackLevelF;
extern volatile float g_lineReverseOrderF;
extern volatile float g_lineTurnSign;
extern volatile float g_lineKp;
extern volatile float g_lineKd;
extern volatile float g_lineTurnLimit;
extern volatile float g_lineFilterAlpha;

extern volatile int16_t g_lineError;
extern volatile uint8_t g_lineValid;
extern volatile uint8_t g_lineMask;
extern volatile uint8_t g_lineRawMask;
extern volatile int8_t g_lastLineDir;
extern volatile uint16_t g_lineLostMs;

static volatile float g_lineErrorFiltered = 0.0f;
static volatile float g_lineLastCtrlError = 0.0f;

static float App_Line_LimitFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

void App_Line_GPIOForceInit(void)
{
    Grayscale_Init();
}

void App_Line_ResetState(void)
{
    g_lineLostMs = 0;
    g_lineErrorFiltered = 0.0f;
    g_lineLastCtrlError = 0.0f;
}

void App_Line_Update(void)
{
    uint8_t raw[GRAYSCALE_CHANNELS];
    static const int16_t weight[GRAYSCALE_CHANNELS] = {-350, -250, -150, -50, 50, 150, 250, 350};
    int32_t sum = 0;
    int16_t count = 0;
    uint8_t mask = 0;
    uint8_t i;
    uint8_t blackLevel;
    uint8_t reverseOrder;

    blackLevel = (g_lineBlackLevelF <= 0.5f) ? 0U : 1U;
    reverseOrder = (g_lineReverseOrderF <= 0.5f) ? 0U : 1U;

    Grayscale_ReadAll(raw);

    g_lineRawMask = 0;
    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (raw[i] == blackLevel) g_lineRawMask |= (uint8_t)(1U << i);
    }

    for (i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        uint8_t physicalIndex;
        uint8_t isBlack;

        physicalIndex = reverseOrder ? (uint8_t)(GRAYSCALE_CHANNELS - 1U - i) : i;
        isBlack = (raw[physicalIndex] == blackLevel) ? 1U : 0U;

        if (isBlack)
        {
            mask |= (uint8_t)(1U << i);
            sum += weight[i];
            count++;
        }
    }

    g_lineMask = mask;

    if (count > 0)
    {
        float rawError;
        float alpha;

        g_lineValid = 1;
        rawError = (float)(sum / count);
        alpha = App_Line_LimitFloat(g_lineFilterAlpha, 0.0f, 1.0f);
        g_lineErrorFiltered = g_lineErrorFiltered * (1.0f - alpha) + rawError * alpha;
        g_lineError = (int16_t)g_lineErrorFiltered;
        g_lastLineDir = (g_lineError >= 0) ? 1 : -1;
        g_lineLostMs = 0;
    }
    else
    {
        g_lineValid = 0;
        if (g_lineLostMs < 60000U) g_lineLostMs += APP_LINE_CONTROL_PERIOD_MS;
    }
}

float App_Line_CalcTurnCmd(void)
{
    float error;
    float dError;
    float turn;

    error = (float)g_lineError;
    dError = error - g_lineLastCtrlError;
    g_lineLastCtrlError = error;

    turn = (-g_lineTurnSign) * (error * g_lineKp + dError * g_lineKd);

    return App_Line_LimitFloat(turn, -g_lineTurnLimit, g_lineTurnLimit);
}
