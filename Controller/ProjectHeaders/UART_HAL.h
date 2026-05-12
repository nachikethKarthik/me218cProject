#ifndef UART_HAL_H
#define UART_HAL_H

#include <stdint.h>
#include <stdbool.h>

void UARTHAL_Init(void);
void UARTHAL_SendByte(uint8_t data);
void UARTHAL_SendBytes(const uint8_t *data, uint16_t length);
bool UARTHAL_IsRxAvailable(void);
bool UARTHAL_ReadByte(uint8_t *data);
void UARTHAL_ClearRxBuffer(void);

#endif