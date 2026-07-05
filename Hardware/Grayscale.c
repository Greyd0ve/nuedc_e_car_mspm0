#include "Grayscale.h"
#include "Board_Config.h"

static void Grayscale_SetAddress(uint8_t channel)
{
    if ((channel & 0x01U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    }

    if ((channel & 0x02U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    }

    if ((channel & 0x04U) != 0U)
    {
        DL_GPIO_setPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
    }

    delay_cycles(160U);
}

void Grayscale_Init(void)
{
    DL_GPIO_clearPins(GRAYSCALE_AD0_PORT, GRAYSCALE_AD0_PIN);
    DL_GPIO_clearPins(GRAYSCALE_AD1_PORT, GRAYSCALE_AD1_PIN);
    DL_GPIO_clearPins(GRAYSCALE_AD2_PORT, GRAYSCALE_AD2_PIN);
}

uint8_t Grayscale_RawOUT(void)
{
    return (DL_GPIO_readPins(GRAYSCALE_OUT_PORT, GRAYSCALE_OUT_PIN) != 0U) ? 1U : 0U;
}

uint8_t Grayscale_ReadChannel(uint8_t channel)
{
    uint8_t a;
    uint8_t b;
    uint8_t c;

    Grayscale_SetAddress((uint8_t)(channel & 0x07U));
    a = Grayscale_RawOUT();
    delay_cycles(40U);
    b = Grayscale_RawOUT();
    delay_cycles(40U);
    c = Grayscale_RawOUT();

    return (uint8_t)(((uint16_t)a + (uint16_t)b + (uint16_t)c) >= 2U);
}

void Grayscale_ReadAll(uint8_t raw[GRAYSCALE_CHANNELS])
{
    uint8_t i;

    if (raw == 0)
    {
        return;
    }

    for (i = 0U; i < GRAYSCALE_CHANNELS; i++)
    {
        raw[i] = Grayscale_ReadChannel(i);
    }
}

uint8_t Grayscale_ReadOne(uint8_t channel)
{
    return Grayscale_ReadChannel(channel);
}
