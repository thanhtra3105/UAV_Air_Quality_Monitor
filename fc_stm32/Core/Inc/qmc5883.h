/*
 * QMC5883.h
 *
 *  Created on: Jun 29, 2026
 *      Author: lethanhtra
 */

#ifndef INC_QMC5883_H_
#define INC_QMC5883_H_

#include "stm32h5xx_hal.h"
#include "math.h"
#define QMC5883_ADDR (0x0D<<1)
#define QMC_CONFIG 0x09
#define QMC_REG_DATA_X_LSB   0x00
#define QMC_REG_SET_RESET 0x0B
#define M_PI 3.14159265358979323846

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t offset_x;
	int16_t offset_y;
	int16_t offset_z;
	float declination;
}QMC5883_t;

void QMC5883_Init(I2C_HandleTypeDef *hi2c,QMC5883_t *mag, uint8_t OSR, uint8_t SCALE, uint8_t ODR, uint8_t mode);
void QMC5883_Read(I2C_HandleTypeDef *hi2c, QMC5883_t *mag);

#endif /* INC_QMC5883_H_ */
