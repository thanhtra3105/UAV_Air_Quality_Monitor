/*
 * serial.c
 *
 *  Created on: Jun 30, 2026
 *      Author: lethanhtra
 */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "serial.h"

void Serial_Init(UART_HandleTypeDef *huart, int baudrate)
{
	huart->Init.BaudRate = baudrate;
	HAL_UART_Init(huart);
}


// Hàm in đa năng qua UART
void Serial_printf(UART_HandleTypeDef *huart, const char *format, ...)	// Serial_Printf(&huart1, "Gia tri nhiet do: %d do C\r\n", so_nguyen);
{
    char buffer[128]; // Mảng đệm chứa chuỗi sau khi format. Có thể tăng kích thước nếu chuỗi rất dài.
    va_list args;

    // Bắt đầu lấy các tham số biến đổi
    va_start(args, format);

    // Ghép các tham số vào mảng đệm buffer dựa trên format
    vsnprintf(buffer, sizeof(buffer), format, args);

    // Kết thúc lấy tham số
    va_end(args);

    // Truyền mảng đệm qua UART
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
