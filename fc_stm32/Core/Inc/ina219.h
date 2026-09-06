/*
 * ina219.h
 *
 *  Created on: Jul 8, 2026
 *      Author: lethanhtra
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_
#include "stm32h5xx_hal.h"

#define INA219_ADDR (uint8_t)(0x40<<1)

void INA219_Init(I2C_HandleTypeDef *hi2c);
float INA219_Read(I2C_HandleTypeDef *hi2c);
float INA219_Read_Bus_Voltage(I2C_HandleTypeDef *hi2c);
void INA219_TriggerRead_IT(I2C_HandleTypeDef *hi2c, uint8_t *rx_buf);
float INA219_ProcessData_IT(uint8_t *buf);
#endif /* INC_INA219_H_ */
