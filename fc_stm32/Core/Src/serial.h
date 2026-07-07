/*
 * serial.h
 *
 *  Created on: Jun 30, 2026
 *      Author: lethanhtra
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include "stm32h5xx_hal.h"

void Serial_Init(UART_HandleTypeDef *huart, int baudrate);
void Serial_printf(UART_HandleTypeDef *huart, const char *format, ...);
#endif /* SRC_SERIAL_H_ */
