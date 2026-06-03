#ifndef AM2301B_H
#define AM2301B_H

#include "stm32f1xx_hal.h"

/* --------------------------------------------------
 * I2C Address — co dinh, khong co chan chon
 * -------------------------------------------------- */
#define AM2301B_I2C_ADDR    (0x38 << 1)

/* --------------------------------------------------
 * Command Bytes
 * -------------------------------------------------- */
#define AM2301B_CMD_INIT        0xBE    /* Initialization command            */
#define AM2301B_CMD_INIT_ARG1   0x08    /* Arg1 sau CMD_INIT                 */
#define AM2301B_CMD_INIT_ARG2   0x00    /* Arg2 sau CMD_INIT                 */
#define AM2301B_CMD_TRIGGER     0xAC    /* Trigger Measurement               */
#define AM2301B_CMD_TRIG_ARG1   0x33    /* Arg1 sau CMD_TRIGGER              */
#define AM2301B_CMD_TRIG_ARG2   0x00    /* Arg2 sau CMD_TRIGGER              */

/* --------------------------------------------------
 * Status byte (byte[0] tra ve sau khi doc)
 * bit[3]: Calibration Enable — phai = 1 moi doc
 * bit[7]: Busy — 1 = dang do, 0 = san sang
 * -------------------------------------------------- */
#define AM2301B_STATUS_BUSY     (1 << 7)
#define AM2301B_STATUS_CAL      (1 << 3)

/* --------------------------------------------------
 * I2C handle — defined in main.c
 * Dung I2C2 rieng biet, tranh conflict 0x38 voi AHT21
 * -------------------------------------------------- */
extern I2C_HandleTypeDef hi2c2;

/* --------------------------------------------------
 * Return type
 * -------------------------------------------------- */
typedef enum {
    AM2301B_OK    = 0,
    AM2301B_ERROR = 1,
    AM2301B_BUSY  = 2
} AM2301B_Status;

/* --------------------------------------------------
 * Data struct
 * Luu gia tri da nhan 10 de nhet vao CAN frame
 * Vi du: 36.5 do C → temp_x10 = 365
 *        65.2 %RH  → humi_x10 = 652
 * -------------------------------------------------- */
typedef struct {
    int16_t  temp_x10;  /* Nhiet do x10, don vi: 0.1 do C */
    uint16_t humi_x10;  /* Do am   x10, don vi: 0.1 %RH   */
} AM2301B_Data;

/* --------------------------------------------------
 * Function Prototypes
 * -------------------------------------------------- */
AM2301B_Status AM2301B_Init(void);
AM2301B_Status AM2301B_Read(AM2301B_Data *data);

#endif /* AM2301B_H */