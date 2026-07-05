#include "Serial.h"
#include "Board_Config.h"
#include <stdarg.h>

#define SERIAL_RX_BUF_SIZE 128U

static volatile uint8_t s_rxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t s_rxHead = 0U;
static volatile uint16_t s_rxTail = 0U;
static volatile uint8_t s_rxFlag = 0U;
static volatile uint8_t s_rxData = 0U;
static volatile uint32_t s_rxOverflowCount = 0U;

static uint16_t Serial_NextIndex(uint16_t index)
{
    index++;
    return (index >= SERIAL_RX_BUF_SIZE) ? 0U : index;
}

static void Serial_PushRx(uint8_t byte)
{
    uint16_t next = Serial_NextIndex(s_rxHead);

    if (next == s_rxTail)
    {
        s_rxOverflowCount++;
        return;
    }

    s_rxBuf[s_rxHead] = byte;
    s_rxHead = next;
    s_rxData = byte;
    s_rxFlag = 1U;
}

void Serial_Init(void)
{
    s_rxHead = 0U;
    s_rxTail = 0U;
    s_rxFlag = 0U;
    s_rxData = 0U;
    s_rxOverflowCount = 0U;

    NVIC_ClearPendingIRQ(SERIAL_UART_IRQN);
    NVIC_EnableIRQ(SERIAL_UART_IRQN);
}

void Serial_SendByte(uint8_t byte)
{
    DL_UART_Main_transmitDataBlocking(SERIAL_UART_INST, byte);
}

void Serial_SendArray(const uint8_t *array, uint16_t length)
{
    uint16_t i;

    if (array == 0)
    {
        return;
    }

    for (i = 0U; i < length; i++)
    {
        Serial_SendByte(array[i]);
    }
}

void Serial_SendString(const char *string)
{
    if (string == 0)
    {
        return;
    }

    while (*string != '\0')
    {
        Serial_SendByte((uint8_t)*string);
        string++;
    }
}

void Serial_SendNumber(uint32_t number, uint8_t length)
{
    char buf[11];
    uint8_t i;

    for (i = 0U; i < length && i < 10U; i++)
    {
        buf[length - 1U - i] = (char)('0' + (number % 10U));
        number /= 10U;
    }
    buf[length] = '\0';
    Serial_SendString(buf);
}

void Serial_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }
    if ((uint32_t)len >= sizeof(buffer))
    {
        len = (int)sizeof(buffer) - 1;
    }

    Serial_SendArray((const uint8_t *)buffer, (uint16_t)len);
}

uint8_t Serial_GetRxFlag(void)
{
    return (s_rxHead != s_rxTail) ? 1U : s_rxFlag;
}

uint8_t Serial_GetRxData(void)
{
    uint8_t byte = 0U;

    (void)Serial_ReadByte(&byte);
    return byte;
}

uint8_t Serial_ReadByte(uint8_t *byte)
{
    if (byte == 0 || s_rxHead == s_rxTail)
    {
        return 0U;
    }

    *byte = s_rxBuf[s_rxTail];
    s_rxTail = Serial_NextIndex(s_rxTail);
    s_rxFlag = (s_rxHead != s_rxTail) ? 1U : 0U;
    return 1U;
}

uint32_t Serial_GetRxOverflowCount(void)
{
    return s_rxOverflowCount;
}

void UART1_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(SERIAL_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(SERIAL_UART_INST))
            {
                Serial_PushRx(DL_UART_Main_receiveData(SERIAL_UART_INST));
            }
            break;

        default:
            break;
    }
}
