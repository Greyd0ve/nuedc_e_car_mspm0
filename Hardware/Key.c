#include "Key.h"
#include "Board_Config.h"

static volatile uint8_t s_keyNum = 0U;

void Key_Init(void)
{
    s_keyNum = 0U;
}

uint8_t Key_GetNum(void)
{
    uint8_t key = s_keyNum;

    if (key != 0U)
    {
        s_keyNum = 0U;
    }
    return key;
}

uint8_t Key_GetState(void)
{
    if (DL_GPIO_readPins(KEY_K1_PORT, KEY_K1_PIN) == 0U)
    {
        return 1U;
    }
    if (DL_GPIO_readPins(KEY_K2_PORT, KEY_K2_PIN) == 0U)
    {
        return 2U;
    }
    if (DL_GPIO_readPins(KEY_K3_PORT, KEY_K3_PIN) == 0U)
    {
        return 3U;
    }
    if (DL_GPIO_readPins(KEY_K4_PORT, KEY_K4_PIN) == 0U)
    {
        return 4U;
    }
    return 0U;
}

void Key_Tick(void)
{
    static uint8_t count = 0U;
    static uint8_t currState = 0U;
    static uint8_t prevState = 0U;

    count++;
    if (count >= 20U)
    {
        count = 0U;
        prevState = currState;
        currState = Key_GetState();
        if (currState == 0U && prevState != 0U)
        {
            s_keyNum = prevState;
        }
    }
}
