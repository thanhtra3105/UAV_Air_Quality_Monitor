#ifndef ENS160_H
#define ENS160_H

#include "stm32f1xx_hal.h"

/* --------------------------------------------------
 * ENS160 I2C Address
 * ADD pin noi VCC hoac de ho → 0x53
 * --------------------------------------------------*/
#define ENS160_I2C_ADDR         (0x53 << 1)

/* ENS160 Registers */
#define ENS160_REG_PART_ID      0x00
#define ENS160_REG_OPMODE       0x10
#define ENS160_REG_DATA_STATUS  0x20
#define ENS160_REG_DATA_TVOC    0x22
#define ENS160_REG_DATA_ECO2    0x24

/* ENS160 Operating Modes */
#define ENS160_OPMODE_RESET     0xF0
#define ENS160_OPMODE_IDLE      0x01
#define ENS160_OPMODE_STANDARD  0x02

/* ENS160 Status Bit */
#define ENS160_NEWDAT_BIT       (1 << 1)

/* --------------------------------------------------
 * AHT21 I2C Address — co dinh
 * --------------------------------------------------*/
#define AHT21_I2C_ADDR          (0x38 << 1)

/* AHT21 Commands */
#define AHT21_CMD_INIT          0xBE
#define AHT21_CMD_INIT_ARG1     0x08
#define AHT21_CMD_INIT_ARG2     0x00
#define AHT21_CMD_TRIGGER       0xAC
#define AHT21_CMD_TRIG_ARG1     0x33
#define AHT21_CMD_TRIG_ARG2     0x00

/* AHT21 Status Bits */
#define AHT21_STATUS_BUSY       (1 << 7)
#define AHT21_STATUS_CAL        (1 << 3)

/* --------------------------------------------------
 * I2C handle dung chung — extern tu main.c
 * --------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

/* --------------------------------------------------
 * Return codes
 * --------------------------------------------------*/
typedef enum {
    ENS160_OK     = 0,
    ENS160_ERROR  = 1,
    ENS160_NODATA = 2
} ENS160_Status;

/* --------------------------------------------------
 * Data structs
 * --------------------------------------------------*/

/* ENS160: CO2 va TVOC */
typedef struct {
    uint16_t eco2;   /* ppm  */
    uint16_t tvoc;   /* ppb  */
} ENS160_Data;

/* AHT21: Nhiet do va Do am — scale x10 cho CAN frame */
typedef struct {
    int16_t  temp_x10;   /* Vi du: 36.5C  → 365  */
    uint16_t humi_x10;   /* Vi du: 65.2%  → 652  */
} AHT21_Data;

/* --------------------------------------------------
 * Function Prototypes
 * --------------------------------------------------*/
ENS160_Status ENS160_Init(void);
ENS160_Status ENS160_Read(ENS160_Data *data);

ENS160_Status AHT21_Init(void);
ENS160_Status AHT21_Read(AHT21_Data *data);

#endif /* ENS160_H */