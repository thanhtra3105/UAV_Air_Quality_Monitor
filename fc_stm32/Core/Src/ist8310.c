/**
 ******************************************************************************
 * @file    IST8310.c
 * @brief   Triển khai driver IST8310 (STM32 HAL I2C).
 ******************************************************************************
 */

#include "IST8310.h"
#include <string.h>

#define IST8310_I2C_TIMEOUT   100   /* ms */

/* Ma trận bù cross-axis A (3x3), tính 1 lần trong IST8310_Init() rồi dùng lại
 * mỗi lần IST8310_Read(). A = (M * X)^-1 với M = diag(50,50,50)
 * => A = X^-1 / 50 (xem mục "Cross-Axis Compensation" trong manual, trang 3-4) */
static float s_compMatrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};
static uint8_t s_compMatrixValid = 0;

/* ==================== State machine cho đo non-blocking ==================== */
typedef enum {
    IST8310_STATE_IDLE = 0,     /* Sẵn sàng bắt đầu 1 lần đo mới */
    IST8310_STATE_MEASURING     /* Đã gửi lệnh đo, đang chờ đủ 6ms */
} IST8310_State_t;

#define IST8310_MEAS_TIME_MS   6   /* Thời gian đo tối thiểu cho chế độ 16x average, theo manual */

static IST8310_State_t s_state = IST8310_STATE_IDLE;
static uint32_t s_measStartTick = 0;

/* ==================== Hàm nội bộ ==================== */

static HAL_StatusTypeDef IST8310_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, IST8310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1, IST8310_I2C_TIMEOUT);
}

static HAL_StatusTypeDef IST8310_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, IST8310_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, IST8310_I2C_TIMEOUT);
}

/* Ghép 2 byte Low/High thành int16_t (2's complement), theo đúng thứ tự manual mô tả */
static int16_t IST8310_CombineLE(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

/**
 * @brief  Đọc 12 byte dữ liệu cross-axis (0x9C~0xAD), dựng ma trận X (3x3),
 *         nghịch đảo và nhân với 1/50 để ra ma trận bù A dùng cho các lần đo sau.
 */
static uint8_t IST8310_ComputeCompensationMatrix(I2C_HandleTypeDef *hi2c)
{
    uint8_t raw[12];
    if (IST8310_ReadRegs(hi2c, IST8310_REG_CROSS_AXIS_START, raw, 12) != HAL_OK)
    {
        return IST8310_ERROR;
    }

    /* Thứ tự thanh ghi theo bảng trong manual: Y11,Y12,Y13,Y21,Y22,Y23,Y31,Y32,Y33
     * (9C,9E,A0,A2,A4,A6,A8,AA,AC), mỗi giá trị 2 byte Low rồi High */
    float y[9];
    for (uint8_t i = 0; i < 9; i++)
    {
        int16_t raw16 = IST8310_CombineLE(raw[i * 2], raw[i * 2 + 1]);
        y[i] = (float)raw16 * IST8310_CROSS_AXIS_M;   /* Xab = Yab * 3/20 */
    }

    /* Ma trận X 3x3 */
    float a = y[0], b = y[1], c = y[2];
    float d = y[3], e = y[4], f = y[5];
    float g = y[6], h = y[7], i2 = y[8];

    float det = a * (e * i2 - f * h) - b * (d * i2 - f * g) + c * (d * h - e * g);

    if (det > -1e-6f && det < 1e-6f)
    {
        /* Ma trận suy biến (không nghịch đảo được) -> dùng ma trận đơn vị / 50 làm dự phòng */
        for (uint8_t r = 0; r < 3; r++)
            for (uint8_t col = 0; col < 3; col++)
                s_compMatrix[r][col] = (r == col) ? (1.0f / 50.0f) : 0.0f;

        s_compMatrixValid = 1;
        return IST8310_OK;
    }

    float invDet = 1.0f / det;

    /* Nghịch đảo ma trận X (công thức cofactor), sau đó nhân thêm 1/50 (do M^-1 = I/50) */
    s_compMatrix[0][0] = (e * i2 - f * h) * invDet / 50.0f;
    s_compMatrix[0][1] = (c * h - b * i2) * invDet / 50.0f;
    s_compMatrix[0][2] = (b * f - c * e) * invDet / 50.0f;

    s_compMatrix[1][0] = (f * g - d * i2) * invDet / 50.0f;
    s_compMatrix[1][1] = (a * i2 - c * g) * invDet / 50.0f;
    s_compMatrix[1][2] = (c * d - a * f) * invDet / 50.0f;

    s_compMatrix[2][0] = (d * h - e * g) * invDet / 50.0f;
    s_compMatrix[2][1] = (b * g - a * h) * invDet / 50.0f;
    s_compMatrix[2][2] = (a * e - b * d) * invDet / 50.0f;

    s_compMatrixValid = 1;
    return IST8310_OK;
}

/* ==================== API ==================== */

uint8_t IST8310_Init(I2C_HandleTypeDef *hi2c, IST8310_Data_t *magData)
{
    uint8_t whoami = 0;

    if (magData == NULL || hi2c == NULL)
    {
        return IST8310_ERROR;
    }

    memset(magData, 0, sizeof(IST8310_Data_t));

    /* 1) Kiểm tra sự hiện diện của chip qua WHO_AM_I */
    if (IST8310_ReadRegs(hi2c, IST8310_REG_WAI, &whoami, 1) != HAL_OK)
    {
        return IST8310_ERROR;
    }
    if (whoami != IST8310_WAI_VALUE)
    {
        return IST8310_ERROR_WAI;
    }

    /* 2) Soft reset, chờ chip ổn định */
    if (IST8310_WriteReg(hi2c, IST8310_REG_CNTL2, IST8310_CNTL2_SRST) != HAL_OK)
    {
        return IST8310_ERROR;
    }
    HAL_Delay(10);

    /* 3) Cấu hình theo đúng manual (phải set ở chế độ Stand-by, tức ngay sau reset) */
    if (IST8310_WriteReg(hi2c, IST8310_REG_AVGCNTL, IST8310_AVGCNTL_16X) != HAL_OK)
    {
        return IST8310_ERROR;
    }
    if (IST8310_WriteReg(hi2c, IST8310_REG_PDCNTL, IST8310_PDCNTL_PULSE) != HAL_OK)
    {
        return IST8310_ERROR;
    }

    /* 4) Đọc dữ liệu cross-axis 1 lần để dựng ma trận bù, dùng lại cho mọi lần Read sau */
    if (IST8310_ComputeCompensationMatrix(hi2c) != IST8310_OK)
    {
        return IST8310_ERROR;
    }

    return IST8310_OK;
}

uint8_t IST8310_Read(I2C_HandleTypeDef *hi2c, IST8310_Data_t *magData)
{
    if (magData == NULL || hi2c == NULL)
    {
        return IST8310_ERROR;
    }

    /* ---- Trạng thái IDLE: bắt đầu 1 lần đo mới, KHÔNG chờ ở đây ---- */
    if (s_state == IST8310_STATE_IDLE)
    {
        if (IST8310_WriteReg(hi2c, IST8310_REG_CNTL1, IST8310_CNTL1_SINGLE_MEAS) != HAL_OK)
        {
            return IST8310_ERROR;
        }
        s_measStartTick = HAL_GetTick();
        s_state = IST8310_STATE_MEASURING;
        return IST8310_BUSY;   /* Dữ liệu chưa sẵn sàng, gọi lại ở (các) vòng lặp sau */
    }

    /* ---- Trạng thái MEASURING: kiểm tra đã đủ 6ms chưa (chỉ so tick, không I2C) ---- */
    if ((HAL_GetTick() - s_measStartTick) < IST8310_MEAS_TIME_MS)
    {
        return IST8310_BUSY;
    }

    /* ---- Đủ thời gian: đọc dữ liệu, áp dụng bù cross-axis, rồi quay về IDLE ---- */
    uint8_t rawData[6];
    if (IST8310_ReadRegs(hi2c, IST8310_REG_DATA_XL, rawData, 6) != HAL_OK)
    {
        s_state = IST8310_STATE_IDLE;   /* Thử lại từ đầu ở lần gọi kế tiếp */
        return IST8310_ERROR;
    }

    magData->raw_x = IST8310_CombineLE(rawData[0], rawData[1]);
    magData->raw_y = IST8310_CombineLE(rawData[2], rawData[3]);
    magData->raw_z = IST8310_CombineLE(rawData[4], rawData[5]);

    /* Áp dụng ma trận bù cross-axis: output = A * raw  (đơn vị: mGauss) */
    if (!s_compMatrixValid)
    {
        /* Chưa gọi Init hoặc tính ma trận thất bại -> trả về dữ liệu thô chưa bù */
        magData->mag_x = (float)magData->raw_x;
        magData->mag_y = (float)magData->raw_y;
        magData->mag_z = (float)magData->raw_z;
    }
    else
    {
        float rx = (float)magData->raw_x;
        float ry = (float)magData->raw_y;
        float rz = (float)magData->raw_z;

        magData->mag_x = s_compMatrix[0][0] * rx + s_compMatrix[0][1] * ry + s_compMatrix[0][2] * rz;
        magData->mag_y = s_compMatrix[1][0] * rx + s_compMatrix[1][1] * ry + s_compMatrix[1][2] * rz;
        magData->mag_z = s_compMatrix[2][0] * rx + s_compMatrix[2][1] * ry + s_compMatrix[2][2] * rz;
    }

    s_state = IST8310_STATE_IDLE;   /* Sẵn sàng bắt đầu lần đo tiếp theo ở lần gọi kế tiếp */
    return IST8310_OK;
}
