/*
 * tof.h
 *
 *  Created on: Jul 3, 2026
 *      Author: lethanhtra
 */

/* GY-TOF10 with I2C*/

#include "tof.h"

int readTOF(I2C_HandleTypeDef *hi2c) {
	uint8_t buffer[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, TOF_ADDR, 0x08, I2C_MEMADD_SIZE_8BIT, buffer, 2, 10);
	if(status != HAL_OK)
		return -1;
	return ((buffer[0] << 8) | buffer[1]);  // mm
}
