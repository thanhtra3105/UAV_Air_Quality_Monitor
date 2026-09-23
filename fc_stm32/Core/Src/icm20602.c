/*
 * icm20602.c
 */

#include "icm20602.h"
#include <string.h>
#include "notch_lpf_filter.h"

IMU_FilterChain_t g_imu_fc;


void IMU_FilterChain_Setup(void) {
	IMU_FilterChain_Init(&g_imu_fc, 8000.0f,   // sample rate 8kHz
			150.0f, 8.0f,     // notch freq + Q — chỉnh theo FFT thực tế của bạn
			70.0f,                 // gyro LPF cutoff
			25.0f);                // accel LPF cutoff
}


uint8_t ICM20602_ReadReg(uint8_t reg) {
	uint8_t tx[2], rx[2];
	tx[0] = reg | 0x80;
	tx[1] = 0xFF;

	ICM_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();

	return rx[1];
}

void ICM20602_WriteReg(uint8_t reg, uint8_t value) {
	uint8_t tx[2], rx[2];
	tx[0] = reg & 0x7F;
	tx[1] = value;

	ICM_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
	ICM_CS_HIGH();
}

void ICM20602_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t tx[32], rx[32];
	tx[0] = reg | 0x80;
	memset(&tx[1], 0xFF, len);

	ICM_CS_LOW();
	HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + 1, HAL_MAX_DELAY);
	ICM_CS_HIGH();

	memcpy(buf, &rx[1], len);
}

/* ============================================================
 * DMA read path — non-blocking
 * ============================================================ */

static volatile uint8_t imu_dma_busy = 0;
static uint8_t icm20602_dma_tx_buf[15];
static uint8_t icm20602_dma_rx_buf[15];
static ICM20602_t *g_imu_ptr = NULL;

void ICM20602_StartReadDMA(ICM20602_t *imu) {
	if (imu_dma_busy) {
		imu->overrun_count++;
		return;
	}

	g_imu_ptr = imu;

	icm20602_dma_tx_buf[0] = ACCEL_XOUT_H | 0x80;
	memset(&icm20602_dma_tx_buf[1], 0xFF, 14);

	imu_dma_busy = 1;
	ICM_CS_LOW();

	if (HAL_SPI_TransmitReceive_DMA(&hspi1, icm20602_dma_tx_buf, icm20602_dma_rx_buf, 15)
			!= HAL_OK) {
		ICM_CS_HIGH();
		imu_dma_busy = 0;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance != SPI1)
		return;

	ICM_CS_HIGH();

	if (g_imu_ptr != NULL) {
		int16_t ax = (int16_t) ((icm20602_dma_rx_buf[1] << 8) | icm20602_dma_rx_buf[2]);
		int16_t ay = (int16_t) ((icm20602_dma_rx_buf[3] << 8) | icm20602_dma_rx_buf[4]);
		int16_t az = (int16_t) ((icm20602_dma_rx_buf[5] << 8) | icm20602_dma_rx_buf[6]);
		int16_t temp = (int16_t) ((icm20602_dma_rx_buf[7] << 8) | icm20602_dma_rx_buf[8]);
		int16_t gx = (int16_t) ((icm20602_dma_rx_buf[9] << 8) | icm20602_dma_rx_buf[10]);
		int16_t gy = (int16_t) ((icm20602_dma_rx_buf[11] << 8) | icm20602_dma_rx_buf[12]);
		int16_t gz = (int16_t) ((icm20602_dma_rx_buf[13] << 8) | icm20602_dma_rx_buf[14]);

		g_imu_ptr->accel.x = (float) ax / 2048.0f;   // ±16g
		g_imu_ptr->accel.y = (float) ay / 2048.0f;
		g_imu_ptr->accel.z = (float) az / 2048.0f;

		g_imu_ptr->gyro.x = (float) gx / 16.4f;      // ±2000dps
		g_imu_ptr->gyro.y = (float) gy / 16.4f;
		g_imu_ptr->gyro.z = (float) gz / 16.4f;

		g_imu_ptr->temperature = ((float) temp / 326.8f) + 25.0f;

		IMU_FilterChain_Push(&g_imu_fc, g_imu_ptr->gyro.x, g_imu_ptr->gyro.y,
				g_imu_ptr->gyro.z, g_imu_ptr->accel.x, g_imu_ptr->accel.y,
				g_imu_ptr->accel.z);
	}

	imu_dma_busy = 0;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance != SPI1)
		return;

	ICM_CS_HIGH();
	imu_dma_busy = 0;
}

/* ============================================================
 * Init — unchanged, still blocking (fine, only runs once at boot)
 * ============================================================ */

void ICM20602_Init(void) {
	HAL_Delay(100);

	ICM20602_WriteReg(PWR_MGMT_1, 0x01);
	HAL_Delay(10);

	ICM20602_WriteReg(PWR_MGMT_2, 0x00);
	ICM20602_WriteReg(SMPLRT_DIV, 0);
	ICM20602_WriteReg(ICM_CONFIG, 0x00); // gyro DLPF_CFG=0: BW 250Hz, Rate 8kHz
	ICM20602_WriteReg(GYRO_CONFIG, 0x18); // ±2000dps, FCHOICE_B=00 (DLPF active)
	ICM20602_WriteReg(ACCEL_CONFIG, 0x18); // ±16g
	ICM20602_WriteReg(ACCEL_CONFIG2, 0x08); // ACCEL_FCHOICE_B=1: bypass DLPF, BW 1046Hz, Rate 4kHz
}
