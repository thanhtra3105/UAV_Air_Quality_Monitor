#ifndef UART_CMD_H
#define UART_CMD_H

#include "main.h"

#define UART_RX_BUFFER_SIZE 64

extern uint8_t uart_rx_char;

void UART_Command_Init(UART_HandleTypeDef *huart);
void UART_CMD_Process(UART_HandleTypeDef *huart);
static void ParseCommand(UART_HandleTypeDef *huart, char *cmd);
#endif
