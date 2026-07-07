#include "qmc5883.h"

int16_t max_x = -32768;
int16_t min_x = 32767;

int16_t max_y = -32768;
int16_t min_y = 32767;

int16_t max_z = -32768;
int16_t min_z = 32767;

void QMC5883_CalibHardIron(QMC5883_t *mag) {

	max_x = 1796;
	min_x = -1286;
	mag->offset_x = (min_x + max_x) / 2;

	max_y = 1357;
	min_y = -1555;
	mag->offset_y = (min_y + max_y) / 2;

	max_z = 1198;
	min_z = -1198;
	mag->offset_z = (min_z + max_z) / 2;
}
void QMC5883_Init(I2C_HandleTypeDef *hi2c,QMC5883_t *mag, uint8_t OSR, uint8_t SCALE,
		uint8_t ODR, uint8_t mode) {
	// set declination

	uint8_t set_reset = 0x01;
	HAL_I2C_Mem_Write(hi2c, QMC5883_ADDR, 0x0B, I2C_MEMADD_SIZE_8BIT,
			&set_reset, 1, HAL_MAX_DELAY);
	// 2. Ghi các thanh ghi nội bộ (rất quan trọng để ổn định trục Y)
	uint8_t reg_20 = 0x40;
	HAL_I2C_Mem_Write(hi2c, QMC5883_ADDR, 0x20, I2C_MEMADD_SIZE_8BIT, &reg_20,
			1, HAL_MAX_DELAY);
	uint8_t reg_21 = 0x01;
	HAL_I2C_Mem_Write(hi2c, QMC5883_ADDR, 0x21, I2C_MEMADD_SIZE_8BIT, &reg_21,
			1, HAL_MAX_DELAY);

	uint8_t cmd = OSR << 6 | SCALE << 4 | ODR << 2 | mode;	// cmd = 0x1D;
	HAL_I2C_Mem_Write(hi2c, QMC5883_ADDR, QMC_CONFIG, I2C_MEMADD_SIZE_8BIT,
			&cmd, 1, HAL_MAX_DELAY);

	HAL_Delay(10);
	QMC5883_CalibHardIron(mag);

}

void QMC5883_Read(I2C_HandleTypeDef *hi2c, QMC5883_t *mag) {
	uint8_t buffer[6];
	uint8_t status;

	HAL_I2C_Mem_Read(hi2c,
	QMC5883_ADDR, 0x06,
	I2C_MEMADD_SIZE_8BIT, &status, 1,
	HAL_MAX_DELAY);

	if (!(status & 0x01))
		return;

	HAL_I2C_Mem_Read(hi2c,
	QMC5883_ADDR, QMC_REG_DATA_X_LSB,
	I2C_MEMADD_SIZE_8BIT, buffer, 6,
	HAL_MAX_DELAY);
	int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
	int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
	int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

	mag->x = (int16_t)(raw_x - mag->offset_x)*0.907;
	mag->y = (int16_t)(raw_y - mag->offset_y)*0.960;
	mag->z = (int16_t)(raw_z - mag->offset_z)*1.167;
//	mag->x = raw_x;
//	mag->y = raw_y;
//	mag->z = raw_z;
}



/*
 CALIB IN MAIN.C
 int16_t max_x = -32768;
	int16_t min_x = 32767;

	int16_t max_y = -32768;
	int16_t min_y = 32767;

	int16_t max_z = -32768;
	int16_t min_z = 32767;

	QMC5883_Read(&hi2c2, &mag);
		if (mag.x > max_x)
			max_x = mag.x;
		if (mag.x < min_x)
			min_x = mag.x;

		if (mag.y > max_y)
			max_y = mag.y;
		if (mag.y < min_y)
			min_y = mag.y;

		if (mag.z > max_z)
			max_z = mag.z;
		if (mag.z < min_z)
			min_z = mag.z;

		Serial_printf(&huart1, "max_x=%d,", max_x);
		Serial_printf(&huart1, "max_y=%d,", max_y);
		Serial_printf(&huart1, "max_z=%d,", max_z);
		Serial_printf(&huart1, "min_x=%d,", min_x);
		Serial_printf(&huart1, "min_y=%d,", min_y);
		Serial_printf(&huart1, "min_z=%d\r\n", min_z);QMC5883_Read(&hi2c2, &mag);
		if (mag.x > max_x)
			max_x = mag.x;
		if (mag.x < min_x)
			min_x = mag.x;

		if (mag.y > max_y)
			max_y = mag.y;
		if (mag.y < min_y)
			min_y = mag.y;

		if (mag.z > max_z)
			max_z = mag.z;
		if (mag.z < min_z)
			min_z = mag.z;

		Serial_printf(&huart1, "max_x=%d,", max_x);
		Serial_printf(&huart1, "max_y=%d,", max_y);
		Serial_printf(&huart1, "max_z=%d,", max_z);
		Serial_printf(&huart1, "min_x=%d,", min_x);
		Serial_printf(&huart1, "min_y=%d,", min_y);
		Serial_printf(&huart1, "min_z=%d\r\n", min_z);
 */
