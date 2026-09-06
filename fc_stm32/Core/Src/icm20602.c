/*
 * icm20602.c
 *
 *  Created on: Jun 29, 2026
 *      Author: lethanhtra
 */

#include "icm20602.h"

static uint8_t ICM_SPI_RW(uint8_t data)
{
    uint8_t rx;

    HAL_SPI_TransmitReceive(&hspi1,&data,&rx,1,100);

    return rx;
}

uint8_t ICM20602_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg | 0x80;
    tx[1] = 0xFF;

    ICM_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    ICM_CS_HIGH();

    return rx[1];
}

void ICM20602_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = value;

    ICM_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    ICM_CS_HIGH();
}
void ICM20602_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[32];
    uint8_t rx[32];

    tx[0] = reg | 0x80;

    memset(&tx[1], 0xFF, len);

    ICM_CS_LOW();

    HAL_SPI_TransmitReceive(&hspi1,
                            tx,
                            rx,
                            len + 1,
                            HAL_MAX_DELAY);

    ICM_CS_HIGH();

    memcpy(buf, &rx[1], len);
}

void ICM20602_Read(ICM20602_t *imu)
{
    uint8_t buf[14];

    int16_t ax,ay,az;
    int16_t gx,gy,gz;
    int16_t temp;

    ICM20602_ReadRegs(ACCEL_XOUT_H,buf,14);

    ax=(buf[0]<<8)|buf[1];
    ay=(buf[2]<<8)|buf[3];
    az=(buf[4]<<8)|buf[5];

    temp=(buf[6]<<8)|buf[7];

    gx=(buf[8]<<8)|buf[9];
    gy=(buf[10]<<8)|buf[11];
    gz=(buf[12]<<8)|buf[13];

    /* ±8g */

    imu->accel.x=(float)ax/4096.0f;
    imu->accel.y=(float)ay/4096.0f;
    imu->accel.z=(float)az/4096.0f;

    /* ±2000dps */

    imu->gyro.x=(float)gx/16.4f;
    imu->gyro.y=(float)gy/16.4f;
    imu->gyro.z=(float)gz/16.4f;

    imu->temperature=((float)temp/326.8f)+25.0f;
}

void ICM20602_Init(void)
{
    HAL_Delay(100);

    /* Wake up */
    ICM20602_WriteReg(PWR_MGMT_1,0x01);

    HAL_Delay(10);

    /* Enable Accel + Gyro */
    ICM20602_WriteReg(PWR_MGMT_2,0x00);

    /* Sample rate */
    ICM20602_WriteReg(SMPLRT_DIV,0);

    /* DLPF */
    ICM20602_WriteReg(ICM_CONFIG,0x03);

    /* Gyro ±2000dps */
    ICM20602_WriteReg(GYRO_CONFIG,0x18);

    /* Accel ±8g */
    ICM20602_WriteReg(ACCEL_CONFIG,0x10);

    ICM20602_WriteReg(ACCEL_CONFIG2,0x03);
}
