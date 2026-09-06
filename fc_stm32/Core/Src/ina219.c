/*
 * ina219.c
 *
 *  Created on: Jul 8, 2026
 *      Author: lethanhtra
 */

#include "ina219.h"

void INA219_Init(I2C_HandleTypeDef *hi2c) {
	uint16_t config_value = 0x3FFF;
	uint8_t data[2];

	// INA219 yêu cầu MSB (byte cao) gửi trước, LSB (byte thấp) gửi sau
	data[0] = (config_value >> 8) & 0xFF;
	data[1] = config_value & 0xFF;

	// Ghi cấu hình vào thanh ghi 0x00
	HAL_I2C_Mem_Write(hi2c, INA219_ADDR, 0x00,
	I2C_MEMADD_SIZE_8BIT, data, 2,
	HAL_MAX_DELAY);
}

float INA219_Read(I2C_HandleTypeDef *hi2c) {
	uint8_t data[2];

	if (HAL_I2C_Mem_Read(hi2c, INA219_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT, data, 2,
	HAL_MAX_DELAY) != HAL_OK) {
		return -1.0f;
	}

	uint16_t raw = ((uint16_t) data[0] << 8) | data[1];
	raw >>= 3;
	return raw*0.004f;
}

float INA219_Read_Bus_Voltage(I2C_HandleTypeDef *hi2c) {
	uint8_t data[2];
	uint16_t reg_value;
	float bus_voltage = 0.0f;

	// Đọc 2 byte từ thanh ghi Bus Voltage (0x02)
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c,
	INA219_ADDR, 0x02,
	I2C_MEMADD_SIZE_8BIT, data, 2,
	HAL_MAX_DELAY);

	if (status == HAL_OK) {
		// INA219 truyền byte cao (MSB) trước, byte thấp (LSB) sau
		reg_value = (data[0] << 8) | data[1];
		// Bỏ qua 3 bit cuối (trạng thái OVF, CNVR...) bằng cách dịch phải 3 bit
		reg_value >>= 3;
		// Mỗi đơn vị (LSB) tương ứng với 4mV (0.004V)
		bus_voltage = reg_value * 0.004f;
		return bus_voltage;
	}

	// Xử lý khi lỗi giao tiếp I2C
	return -1.0f;
}


void INA219_TriggerRead_IT(I2C_HandleTypeDef *hi2c, uint8_t *rx_buf) {
    // Ra lệnh đọc 2 byte từ thanh ghi 0x02 bằng ngắt
    HAL_I2C_Mem_Read_IT(hi2c, INA219_ADDR, 0x02, I2C_MEMADD_SIZE_8BIT, rx_buf, 2);
}

float INA219_ProcessData_IT(uint8_t *buf) {
    uint16_t reg_value = (buf[0] << 8) | buf[1];
    reg_value >>= 3;
    return reg_value * 0.004f;
}
