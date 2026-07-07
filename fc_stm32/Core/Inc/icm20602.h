#ifndef INC_ICM20602_H_
#define INC_ICM20602_H_

#include "stm32h5xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/* GPIO */
#define ICM_CS_PORT    GPIOC
#define ICM_CS_PIN     GPIO_PIN_5

#define ICM_CS_LOW()   HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_RESET)
#define ICM_CS_HIGH()  HAL_GPIO_WritePin(ICM_CS_PORT,ICM_CS_PIN,GPIO_PIN_SET)

/* Register */
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define ACCEL_XOUT_H    0x3B

typedef struct
{
    float x;
    float y;
    float z;
}Vector3f;

typedef struct
{
    Vector3f accel;
    Vector3f gyro;
    float temperature;
}ICM20602_t;

uint8_t ICM20602_ReadReg(uint8_t reg);
void ICM20602_WriteReg(uint8_t reg,uint8_t value);
void ICM20602_ReadRegs(uint8_t reg,uint8_t *buf,uint8_t len);

void ICM20602_Init(void);

void ICM20602_Read(ICM20602_t *imu);

#endif
