/*
 * tof.h
 *
 *  Created on: Jul 3, 2026
 *      Author: lethanhtra
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "stm32h5xx_hal.h"

#define TOF_ADDR 0xA4

int readTOF(I2C_HandleTypeDef *hi2c);
#endif /* INC_TOF_H_ */
