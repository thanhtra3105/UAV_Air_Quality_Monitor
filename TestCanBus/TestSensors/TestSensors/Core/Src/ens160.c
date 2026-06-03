#include "ens160.h"

/* ======================================================
 * PHAN 1 — ENS160 (CO2, TVOC)
 * Giao tiep I2C1, dia chi 0x53
 * ====================================================== */

/* ------------------------------------------------------
 * Ghi 1 byte vao thanh ghi ENS160
 * buf[0] = dia chi thanh ghi
 * buf[1] = gia tri can ghi
 * ------------------------------------------------------ */
static ENS160_Status ENS160_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    if (HAL_I2C_Master_Transmit(&hi2c1, ENS160_I2C_ADDR, buf, 2, 100) != HAL_OK)
        return ENS160_ERROR;
    return ENS160_OK;
}

/* ------------------------------------------------------
 * Doc n byte tu thanh ghi ENS160
 * Gui dia chi thanh ghi truoc, sau do doc du lieu
 * ------------------------------------------------------ */
static ENS160_Status ENS160_ReadReg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (HAL_I2C_Master_Transmit(&hi2c1, ENS160_I2C_ADDR, &reg, 1, 100) != HAL_OK)
        return ENS160_ERROR;
    if (HAL_I2C_Master_Receive(&hi2c1, ENS160_I2C_ADDR, buf, len, 100) != HAL_OK)
        return ENS160_ERROR;
    return ENS160_OK;
}

/* ------------------------------------------------------
 * ENS160_Init
 * Sequence khoi dong:
 *   1. Doc PART_ID kiem tra ket noi vat ly (phai = 0x0160)
 *   2. Reset chip
 *   3. Chuyen sang Idle mode
 *   4. Chuyen sang Standard mode → bat dau do
 * ------------------------------------------------------ */
ENS160_Status ENS160_Init(void)
{
    uint8_t id[2];

    /* Kiem tra PART_ID = 0x0160
     * Little endian: byte[0]=0x60, byte[1]=0x01 */
    if (ENS160_ReadReg(ENS160_REG_PART_ID, id, 2) != ENS160_OK)
        return ENS160_ERROR;
    if (id[0] != 0x60 || id[1] != 0x01)
        return ENS160_ERROR;    /* Sai chip hoac mat ket noi I2C */

    /* Reset chip */
    if (ENS160_WriteReg(ENS160_REG_OPMODE, ENS160_OPMODE_RESET) != ENS160_OK)
        return ENS160_ERROR;
    HAL_Delay(100);

    /* Idle mode — can thiet truoc khi sang Standard */
    if (ENS160_WriteReg(ENS160_REG_OPMODE, ENS160_OPMODE_IDLE) != ENS160_OK)
        return ENS160_ERROR;
    HAL_Delay(10);

    /* Standard mode — chip bat dau do CO2 va TVOC */
    if (ENS160_WriteReg(ENS160_REG_OPMODE, ENS160_OPMODE_STANDARD) != ENS160_OK)
        return ENS160_ERROR;
    HAL_Delay(50);

    return ENS160_OK;
}

/* ------------------------------------------------------
 * ENS160_Read
 * Kiem tra DATA_STATUS truoc khi doc:
 *   - NEWDAT bit = 1 → co data moi → doc tiep
 *   - NEWDAT bit = 0 → chua co data → tra ve NODATA
 *     (goi lai sau 1 giay, giu nguyen gia tri cu)
 *
 * Chip can ~3 phut warm up sau power-on
 * moi co gia tri CO2/TVOC on dinh
 * ------------------------------------------------------ */
ENS160_Status ENS160_Read(ENS160_Data *data)
{
    uint8_t status;
    uint8_t raw[2];

    /* Doc DATA_STATUS */
    if (ENS160_ReadReg(ENS160_REG_DATA_STATUS, &status, 1) != ENS160_OK)
        return ENS160_ERROR;

    /* Kiem tra NEWDAT bit */
    if (!(status & ENS160_NEWDAT_BIT))
        return ENS160_NODATA;   /* Chua co data moi */

    /* Doc TVOC — 2 byte, Little Endian, don vi ppb */
    if (ENS160_ReadReg(ENS160_REG_DATA_TVOC, raw, 2) != ENS160_OK)
        return ENS160_ERROR;
    data->tvoc = (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));

    /* Doc eCO2 — 2 byte, Little Endian, don vi ppm */
    if (ENS160_ReadReg(ENS160_REG_DATA_ECO2, raw, 2) != ENS160_OK)
        return ENS160_ERROR;
    data->eco2 = (uint16_t)(raw[0] | ((uint16_t)raw[1] << 8));

    return ENS160_OK;
}


/* ======================================================
 * PHAN 2 — AHT21 (Nhiet do, Do am)
 * Cung bus I2C1, dia chi 0x38
 * Khong conflict voi ENS160 vi khac dia chi
 * ====================================================== */

/* ------------------------------------------------------
 * AHT21_Init
 * Sequence:
 *   1. Doc status byte
 *   2. Neu CAL bit = 0 → gui lenh init 0xBE 0x08 0x00
 *   3. Doc lai status → kiem tra CAL bit da set chua
 *
 * CAL bit = bit3 cua status byte
 * Neu sau init van = 0 → chip loi
 * ------------------------------------------------------ */
ENS160_Status AHT21_Init(void)
{
    uint8_t cmd_init[3] = { AHT21_CMD_INIT, AHT21_CMD_INIT_ARG1, AHT21_CMD_INIT_ARG2 };
    uint8_t status = 0;

    /* Cho chip on dinh sau power-on */
    HAL_Delay(100);

    /* Doc status byte hien tai */
    if (HAL_I2C_Master_Receive(&hi2c1, AHT21_I2C_ADDR, &status, 1, 100) != HAL_OK)
        return ENS160_ERROR;

    /* Kiem tra CAL bit (bit3)
     * Neu chua set → chip chua calibrate → gui lenh init */
    if (!(status & AHT21_STATUS_CAL))
    {
        /* Gui lenh init 0xBE 0x08 0x00 */
        if (HAL_I2C_Master_Transmit(&hi2c1, AHT21_I2C_ADDR, cmd_init, 3, 100) != HAL_OK)
            return ENS160_ERROR;
        HAL_Delay(10);

        /* Doc lai status de xac nhan CAL bit da duoc set */
        if (HAL_I2C_Master_Receive(&hi2c1, AHT21_I2C_ADDR, &status, 1, 100) != HAL_OK)
            return ENS160_ERROR;

        /* Neu van chua co CAL bit → chip bi loi */
        if (!(status & AHT21_STATUS_CAL))
            return ENS160_ERROR;
    }

    return ENS160_OK;
}

/* ------------------------------------------------------
 * AHT21_Read
 * Sequence:
 *   1. Gui lenh Trigger 0xAC 0x33 0x00
 *   2. Doi 80ms (thoi gian do cua chip)
 *   3. Doc 6 byte ket qua
 *   4. Kiem tra BUSY bit → neu con busy thi chua xong
 *   5. Giai ma raw data → scale x10 cho CAN frame
 *
 * Cong thuc tinh (theo datasheet):
 *   Do am (%RH) = humi_raw / 2^20 * 100
 *   Nhiet do (C) = temp_raw / 2^20 * 200 - 50
 *
 * Scale x10 de luu int16/uint16 vao CAN:
 *   humi_x10 = humi_raw * 1000 / 1048576
 *   temp_x10 = temp_raw * 2000 / 1048576 - 500
 *
 * Vi du: 36.5C → temp_x10 = 365
 *        65.2% → humi_x10 = 652
 * ------------------------------------------------------ */
ENS160_Status AHT21_Read(AHT21_Data *data)
{
    uint8_t cmd[3] = { AHT21_CMD_TRIGGER, AHT21_CMD_TRIG_ARG1, AHT21_CMD_TRIG_ARG2 };
    uint8_t rx[6]  = { 0 };

    /* Gui lenh Trigger do */
    if (HAL_I2C_Master_Transmit(&hi2c1, AHT21_I2C_ADDR, cmd, 3, 100) != HAL_OK)
        return ENS160_ERROR;

    /* Cho chip do xong — datasheet yeu cau ~80ms */
    HAL_Delay(80);

    /* Doc 6 byte ket qua:
     * rx[0]        : Status
     * rx[1]        : Humi[19:12]
     * rx[2]        : Humi[11:4]
     * rx[3] bit7-4 : Humi[3:0]
     * rx[3] bit3-0 : Temp[19:16]
     * rx[4]        : Temp[15:8]
     * rx[5]        : Temp[7:0]
     */
    if (HAL_I2C_Master_Receive(&hi2c1, AHT21_I2C_ADDR, rx, 6, 100) != HAL_OK)
        return ENS160_ERROR;

    /* Kiem tra BUSY bit (bit7)
     * Neu = 1 → chip van dang do, data chua san sang */
    if (rx[0] & AHT21_STATUS_BUSY)
        return ENS160_NODATA;

    /* Giai ma humidity raw 20-bit */
    uint32_t humi_raw = ((uint32_t)rx[1] << 12)
                      | ((uint32_t)rx[2] <<  4)
                      | ((uint32_t)rx[3] >>  4);

    /* Giai ma temperature raw 20-bit
     * Lay 4 bit thap cua rx[3] lam bit cao nhat */
    uint32_t temp_raw = (((uint32_t)rx[3] & 0x0F) << 16)
                      |  ((uint32_t)rx[4] <<  8)
                      |   (uint32_t)rx[5];

    /* Tinh gia tri thuc va scale x10 */
    data->humi_x10 = (uint16_t)((humi_raw * 1000UL) / 1048576UL);
    data->temp_x10 = (int16_t) ((temp_raw * 2000UL) / 1048576UL - 500);

    return ENS160_OK;
}