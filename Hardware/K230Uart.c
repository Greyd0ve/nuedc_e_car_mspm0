#include "K230Uart.h"
#include "Board_Config.h"
#include "ti_msp_dl_config.h"
#include "cmsis_compiler.h"

static volatile uint16_t s_rxHead;
static volatile uint16_t s_rxTail;
static uint8_t s_rxBuffer[K230_UART_RX_BUFFER_SIZE];
static volatile uint32_t s_rxByteCount;
static volatile uint32_t s_overflowCount;

void K230Uart_Init(void)
{
    s_rxHead        = 0U;
    s_rxTail        = 0U;
    s_rxByteCount   = 0U;
    s_overflowCount = 0U;
}

uint8_t K230Uart_ReadByte(uint8_t *outByte)
{
    uint16_t head;
    uint32_t primask;

    if (outByte == 0) { return 0U; }

    primask = __get_PRIMASK();
    __disable_irq();
    head = s_rxHead;
    if (head == s_rxTail)
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    *outByte = s_rxBuffer[s_rxTail];
    s_rxTail = (uint16_t)((s_rxTail + 1U) & (K230_UART_RX_BUFFER_SIZE - 1U));

    if (primask == 0U) { __enable_irq(); }
    return 1U;
}

uint16_t K230Uart_Available(void)
{
    uint16_t avail;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    avail = (uint16_t)((s_rxHead - s_rxTail) & (K230_UART_RX_BUFFER_SIZE - 1U));
    if (primask == 0U) { __enable_irq(); }
    return avail;
}

uint32_t K230Uart_GetRxByteCount(void)
{
    return s_rxByteCount;
}

uint32_t K230Uart_GetOverflowCount(void)
{
    return s_overflowCount;
}

void K230Uart_Clear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rxHead        = 0U;
    s_rxTail        = 0U;
    s_rxByteCount   = 0U;
    s_overflowCount = 0U;
    if (primask == 0U) { __enable_irq(); }
}

void UART0_IRQHandler(void)
{
    uint32_t status = DL_UART_Main_getPendingInterrupt(K230_UART_INST);

    if ((status & DL_UART_MAIN_IIDX_RX) != 0U)
    {
        uint8_t byte = DL_UART_Main_receiveData(K230_UART_INST);
        uint16_t nextHead = (uint16_t)((s_rxHead + 1U) & (K230_UART_RX_BUFFER_SIZE - 1U));

        s_rxByteCount++;

        if (nextHead != s_rxTail)
        {
            s_rxBuffer[s_rxHead] = byte;
            s_rxHead = nextHead;
        }
        else
        {
            s_overflowCount++;
        }
    }
}
