/*
#include "am2301b.h"

--------------------------------------------------
 * AM2301B_Init
 *
 * Gui lenh 0xBE 0x08 0x00 de khoi dong calibration.
 * Chip can it nhat 100ms sau power-on truoc khi init.
 * -------------------------------------------------- 
AM2301B_Status AM2301B_Init(void)
{
    uint8_t cmd[3] = { AM2301B_CMD_INIT, AM2301B_CMD_INIT_ARG1, AM2301B_CMD_INIT_ARG2 };

    HAL_Delay(100);     

    if (HAL_I2C_Master_Transmit(&hi2c2, AM2301B_I2C_ADDR, cmd, 3, 100) != HAL_OK)
        return AM2301B_ERROR;

    HAL_Delay(10);
    return AM2301B_OK;
}
--------------------------------------------------
 * AM2301B_Read
 *
 * Sequence:
 *   1. Gui lenh Trigger (0xAC 0x33 0x00)
 *   2. Doi 80ms (thoi gian do cua chip)
 *   3. Doc 6 byte ket qua
 *   4. Kiem tra busy bit
 *   5. Tinh toan Nhiet do & Do am
 *
 * Cong thuc tinh (theo datasheet):
 *   RH  = (raw_humi  / 2^20) * 100
 *   Tmp = (raw_temp  / 2^20) * 200 - 50
 * -------------------------------------------------- 
AM2301B_Status AM2301B_Read(AM2301B_Data *data)
{
    uint8_t cmd[3] = { AM2301B_CMD_TRIGGER, AM2301B_CMD_TRIG_ARG1, AM2301B_CMD_TRIG_ARG2 };
    uint8_t rx[6]  = { 0 };

     Buoc 1: Trigger measurement 
    if (HAL_I2C_Master_Transmit(&hi2c2, AM2301B_I2C_ADDR, cmd, 3, 100) != HAL_OK)
        return AM2301B_ERROR;

    Buoc 2: Cho chip do xong — datasheet yeu cau ~80ms 
    HAL_Delay(80);

     Buoc 3: Doc 6 byte
     * [0]     : Status
     * [1][2][3]: Humidity raw (20-bit, nam o bit 19..0 cua 3 byte)
     * [3][4][5]: Temperature raw (dung chung byte[3])
     *
     * Cu the:
     *   humi_raw = ( rx[1]<<12 | rx[2]<<4 | rx[3]>>4 ) & 0xFFFFF
     *   temp_raw = ( (rx[3]&0x0F)<<16 | rx[4]<<8 | rx[5] )
     
    if (HAL_I2C_Master_Receive(&hi2c2, AM2301B_I2C_ADDR, rx, 6, 100) != HAL_OK)
        return AM2301B_ERROR;

    Buoc 4: Kiem tra busy bit — neu van busy thi data chua san sang 
    if (rx[0] & AM2301B_STATUS_BUSY)
        return AM2301B_BUSY;

    Buoc 5: Giai ma raw data 
    uint32_t humi_raw = ((uint32_t)rx[1] << 12)
                      | ((uint32_t)rx[2] <<  4)
                      | ((uint32_t)rx[3] >>  4);

    uint32_t temp_raw = (((uint32_t)rx[3] & 0x0F) << 16)
                      | ((uint32_t)rx[4] <<  8)
                      |  (uint32_t)rx[5];

    Buoc 6: Tinh gia tri thuc, scale x10 de luu vao int16/uint16
     *
     * Do am (%RH x10):
     *   humi = humi_raw / 1048576.0 * 100.0
     *   → x10: humi_raw * 1000 / 1048576
     *
     * Nhiet do (°C x10):
     *   temp = temp_raw / 1048576.0 * 200.0 - 50.0
     *   → x10: temp_raw * 2000 / 1048576 - 500
     
    data->humi_x10 = (uint16_t)((humi_raw * 1000UL) / 1048576UL);
    data->temp_x10 = (int16_t) ((temp_raw * 2000UL) / 1048576UL - 500);

    return AM2301B_OK;
}
*/