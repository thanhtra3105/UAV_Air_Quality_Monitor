#include "dsp310.h"
#include "math.h"
// --- Định nghĩa thanh ghi nội bộ ---
#define PSR_CONFIG      0x06
#define TEMP_CONFIG     0x07
#define MEAS_CFG        0x08
#define CFG_REG         0x09
#define PROD_ID_REG     0x0D
#define COEF_REG_BASE   0x10
#define TMP_COEF_SRCE   0x28
#define RESULT_BASE     0x00

// Hệ số scale cho Oversampling 16x [cite: 1012]
static const uint32_t kP = 253952;
static const uint32_t kT = 253952;

// --- Hàm Private ---
static void DSP310_WriteReg(DSP310_t *dev, uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(dev->hi2c, DSP310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
}

static uint8_t DSP310_ReadReg(DSP310_t *dev, uint8_t reg) {
    uint8_t value = 0;
    HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
    return value;
}

static uint8_t DSP310_CalibCoefficient(DSP310_t *dev) {
    uint8_t buf[18] = {0};

    // Đọc 18 byte hệ số[cite: 1]
    if (HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, COEF_REG_BASE, I2C_MEMADD_SIZE_8BIT, buf, 18, 50) != HAL_OK) {
        return 0;
    }

    // --- Lấy hệ số cho Nhiệt độ (c0, c1 là số 12-bit) ---
    dev->c0 = ((int32_t)buf[0] << 4) | (buf[1] >> 4);
    if(dev->c0 & (1 << 11)) dev->c0 |= 0xFFFFF000; // Mở rộng dấu

    dev->c1 = ((int32_t)(buf[1] & 0x0F) << 8) | buf[2];
    if(dev->c1 & (1 << 11)) dev->c1 |= 0xFFFFF000;

    // --- Lấy hệ số cho Áp suất (Giữ nguyên như code cũ) ---[cite: 1]
    dev->c00 = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);
    if(dev->c00 & (1 << 19)) dev->c00 |= 0xFFF00000;

    dev->c10 = ((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7];
    if(dev->c10 & (1 << 19)) dev->c10 |= 0xFFF00000;

    dev->c01 = ((int32_t)buf[8] << 8) | buf[9];
    if(dev->c01 & (1 << 15)) dev->c01 |= 0xFFFF0000;

    dev->c11 = ((int32_t)buf[10] << 8) | buf[11];
    if(dev->c11 & (1 << 15)) dev->c11 |= 0xFFFF0000;

    dev->c20 = ((int32_t)buf[12] << 8) | buf[13];
    if(dev->c20 & (1 << 15)) dev->c20 |= 0xFFFF0000;

    dev->c21 = ((int32_t)buf[14] << 8) | buf[15];
    if(dev->c21 & (1 << 15)) dev->c21 |= 0xFFFF0000;

    dev->c30 = ((int32_t)buf[16] << 8) | buf[17];
    if(dev->c30 & (1 << 15)) dev->c30 |= 0xFFFF0000;

    return 1;
}

// --- Hàm Public ---
uint8_t DSP310_Init(DSP310_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    uint8_t status = 0;

    // 0. Test ID để xác nhận chip sống [cite: 1190-1202]
    if (DSP310_ReadReg(dev, PROD_ID_REG) != 0x10) {
        return 0;
    }

    // 1. Chờ SENSOR_RDY và COEF_RDY (Check bit 7 và 6) [cite: 1056-1068]
    for (int i = 0; i < 20; i++) {
        HAL_Delay(10);
        status = DSP310_ReadReg(dev, MEAS_CFG);
        if ((status & 0xC0) == 0xC0) break;
    }

    if ((status & 0xC0) != 0xC0) {
        return 0; // Timeout
    }

    // 2. Đọc hệ số calibration
    if (!DSP310_CalibCoefficient(dev)) {
        return 0;
    }

    // 3. Đọc nguồn cảm biến nhiệt độ [cite: 1239-1241]
    uint8_t tmp_coef_srce = DSP310_ReadReg(dev, TMP_COEF_SRCE) & 0x80;

    // 4. Ghi cấu hình: Rate = 16Hz (0x04), OSR = 16x (0x04) -> Value: 0x44 [cite: 1002, 1039]
    DSP310_WriteReg(dev, PSR_CONFIG, 0x44);
    DSP310_WriteReg(dev, TEMP_CONFIG, tmp_coef_srce | 0x44);

    // Kích hoạt Bit-shift cho OSR > 8x [cite: 1106]
    DSP310_WriteReg(dev, CFG_REG, 0x03 << 2);

    // 5. Kích hoạt BACKGROUND MODE (Đo ngầm liên tục áp suất và nhiệt độ) [cite: 1068]
    DSP310_WriteReg(dev, MEAS_CFG, 0x07);

    return 1;
}

void DSP310_ReadData(DSP310_t *dev) {
    uint8_t buf[6] = {0};

    // Đọc 1 lúc 6 byte kết quả liên tục (3 byte áp suất, 3 byte nhiệt độ)[cite: 1]
    if (HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, RESULT_BASE, I2C_MEMADD_SIZE_8BIT, buf, 6, 20) != HAL_OK) {
        return;
    }

    // Xử lý dữ liệu thô (24-bit bù 2)
    int32_t Praw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if(Praw & 0x800000) Praw |= 0xFF000000;

    int32_t Traw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | buf[5];
    if(Traw & 0x800000) Traw |= 0xFF000000;

    float Praw_sc = (float)Praw / kP;
    float Traw_sc = (float)Traw / kT;

    // 1. Tính toán nhiệt độ bù trừ
    dev->temperature = (float)dev->c0 * 0.5f + (float)dev->c1 * Traw_sc;

    // 2. Tính toán áp suất bù trừ[cite: 1]
    dev->pressure_Pa = (float)dev->c00
                + Praw_sc * ((float)dev->c10 + Praw_sc * ((float)dev->c20 + Praw_sc * (float)dev->c30))
                + Traw_sc * (float)dev->c01
                + Traw_sc * Praw_sc * ((float)dev->c11 + Praw_sc * (float)dev->c21);

    dev->pressure_hPa = dev->pressure_Pa * 0.01f;

    // 3. Tính độ cao[cite: 1]
    dev->altitude = 44330.0f * (1.0f - pow((dev->pressure_hPa / 1013.25f), (1.0f / 5.255f)));
}


// Đọc dữ liệu mới từ DSP310 (non-blocking, không chờ)
// Trả về 1 nếu có dữ liệu mới (đã cập nhật dev->pressure/temperature/altitude)
// Trả về 0 nếu sensor chưa xong conversion (giữ nguyên giá trị cũ)

uint8_t DSP310_Read(DSP310_t *dev) {
    // 1. Kiểm tra cờ sẵn sàng: bit4 = PRS_RDY, bit5 = TMP_RDY
    uint8_t status = DSP310_ReadReg(dev, MEAS_CFG);
    if ((status & 0x30) != 0x30) {
        return 0; // Chưa có data mới (cả P và T), bỏ qua vòng này
    }

    uint8_t buf[6] = {0};

    // 2. Đọc 6 byte kết quả (3 byte áp suất, 3 byte nhiệt độ)
    if (HAL_I2C_Mem_Read(dev->hi2c, DSP310_I2C_ADDR, RESULT_BASE,
                          I2C_MEMADD_SIZE_8BIT, buf, 6, 20) != HAL_OK) {
        return 0; // Lỗi I2C, coi như không có data mới
    }

    // 3. Xử lý dữ liệu thô (24-bit bù 2)
    int32_t Praw = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    if (Praw & 0x800000) Praw |= 0xFF000000;

    int32_t Traw = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | buf[5];
    if (Traw & 0x800000) Traw |= 0xFF000000;

    float Praw_sc = (float)Praw / kP;
    float Traw_sc = (float)Traw / kT;

    // 4. Tính toán nhiệt độ bù trừ
    dev->temperature = (float)dev->c0 * 0.5f + (float)dev->c1 * Traw_sc;

    // 5. Tính toán áp suất bù trừ
    dev->pressure_Pa = (float)dev->c00
                + Praw_sc * ((float)dev->c10 + Praw_sc * ((float)dev->c20 + Praw_sc * (float)dev->c30))
                + Traw_sc * (float)dev->c01
                + Traw_sc * Praw_sc * ((float)dev->c11 + Praw_sc * (float)dev->c21);

    dev->pressure_hPa = dev->pressure_Pa * 0.01f;

    // 6. Tính độ cao
    dev->altitude = 44330.0f * (1.0f - powf((dev->pressure_hPa / 1013.25f), (1.0f / 5.255f)));

    return 1; // Có dữ liệu mới
}
