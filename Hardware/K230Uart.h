#ifndef __K230UART_H
#define __K230UART_H

#include <stdint.h>

#define K230_UART_RX_BUFFER_SIZE 512U

void K230Uart_Init(void);

uint8_t K230Uart_ReadByte(uint8_t *outByte);

uint16_t K230Uart_Available(void);

uint32_t K230Uart_GetRxByteCount(void);

uint32_t K230Uart_GetOverflowCount(void);

void K230Uart_Clear(void);

#endif
