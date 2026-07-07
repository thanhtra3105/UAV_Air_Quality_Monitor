/*
 * uart_cmd.c
 *
 *  Created on: Jul 2, 2026
 *      Author: lethanhtra
 */

#include "uart_cmd.h"
#include "pid_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

extern PIDController_t PID_Angle;
extern PIDController_t PID_Rate_Pitch;
extern PIDController_t PID_Rate_Roll;
extern PIDController_t PID_Rate_Yaw;
extern PIDController_t PID_Alt;
extern PIDController_t PID_Alt_Pos;
extern PIDController_t PID_Alt_Vel;
uint8_t uart_rx_char;
static char rxBuffer[UART_RX_BUFFER_SIZE];
static uint8_t rxIndex = 0;

float kp, ki, kd;
void UART_Command_Init(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(huart, &uart_rx_char, 1);
}

static void ParseCommand(UART_HandleTypeDef *huart, char *cmd) {
	char axis;

	if (sscanf(cmd, "SET,%c,%f,%f,%f", &axis, &kp, &ki, &kd) == 4) {
		switch (axis) {
		case 'P':
			PID_SetGain(&PID_Rate_Pitch, kp, ki, kd);
//                kp_test = kp;
			break;
		case 'R':
			PID_SetGain(&PID_Rate_Roll, kp, ki, kd);
			break;
		case 'Y':
			PID_SetGain(&PID_Rate_Yaw, kp, ki, kd);
			break;
		case 'A':
			PID_SetGain(&PID_Angle, kp, ki, kd);
			break;
		case 'H':
			PID_SetGain(&PID_Alt_Pos, kp, ki, kd);
			break;
		case 'Z':
			PID_SetGain(&PID_Alt_Vel, kp, ki, kd);
			break;
		default:
			return;
		}

		char tx[64];

		sprintf(tx, "OK,%c,%.4f,%.4f,%.4f\r\n", axis, kp, ki, kd);

		HAL_UART_Transmit(huart, (uint8_t*) tx, strlen(tx), 100);
	}
}

void UART_CMD_Process(UART_HandleTypeDef *huart) {
	if (uart_rx_char == '\n') {
		rxBuffer[rxIndex] = 0;

		ParseCommand(huart, rxBuffer);

		rxIndex = 0;
	} else {
		if (rxIndex < UART_RX_BUFFER_SIZE - 1) {
			rxBuffer[rxIndex++] = uart_rx_char;
		}
	}

	HAL_UART_Receive_IT(huart, &uart_rx_char, 1);
}
