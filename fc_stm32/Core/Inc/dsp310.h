/*
 * dsp310.h
 *
 *  Created on: Jul 10, 2026
 *      Author: lethanhtra
 */

/* dsp310.h */
#ifndef INC_DSP310_H_
#define INC_DSP310_H_

#include "stm32h5xx_hal.h"
#define DSP310_I2C_ADDR (0x77 << 1)

// Struct lưu trữ ngữ cảnh, hệ số bù và dữ liệu của cảm biến
typedef struct {
    I2C_HandleTypeDef *hi2c;

    // Hệ số bù[cite: 2]
    int32_t c0, c1;                                // Dành cho nhiệt độ
    int32_t c00, c10, c20, c30, c01, c11, c21;     // Dành cho áp suất

    // Dữ liệu đo đạc trực tiếp
    float temperature;   // Nhiệt độ (oC)
    float pressure_Pa;   // Áp suất (Pascal)
    float pressure_hPa;  // Áp suất (HectoPascal)
    float altitude;      // Độ cao (m)
} DSP310_t;

// Khai báo các hàm giao tiếp
uint8_t DSP310_Init(DSP310_t *dev, I2C_HandleTypeDef *hi2c);
void DSP310_ReadData(DSP310_t *dev); // Hàm mới thay thế cho việc gọi lẻ tẻ
uint8_t DSP310_Read(DSP310_t *dev);
void DSP310_DeInit(DSP310_t *dev);
void DSP310_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void DSP310_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
#endif /* INC_DSP310_H_ */
