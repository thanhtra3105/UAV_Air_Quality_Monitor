/**
 ******************************************************************************
 * @file    IST8310.h
 * @brief   Driver cho cảm biến từ trường (magnetometer) IST8310, dùng STM32 HAL I2C.
 *
 * Tham khảo: IST8310 User Manual v1.5 (iSentek)
 *  - I2C slave address mặc định (CAD0, CAD1 nối GND/floating): 0x0E (7-bit) / 0x1C (8-bit)
 *  - Quy trình đọc dữ liệu (theo manual, mục "Read Process"):
 *      1) Ghi 0x24 vào thanh ghi 0x41 (AVGCNTL)  -> 16x internal average (low noise mode)
 *      2) Ghi 0xC0 vào thanh ghi 0x42 (PDCNTL)   -> Set/Reset pulse duration
 *      3) Ghi 0x01 vào thanh ghi 0x0A (CNTL1)    -> Single Measurement Mode
 *      4) Chờ tối thiểu 6ms
 *      5) Đọc 6 byte liên tiếp từ thanh ghi 0x03 -> XL,XH,YL,YH,ZL,ZH
 *      6) Thực hiện Cross-Axis Compensation (dùng ma trận lấy từ thanh ghi 0x9C~0xAD)
 *
 * Lưu ý: các thanh ghi WHO_AM_I (0x00) và CNTL2 (0x0B) không xuất hiện trong bản
 * manual rút gọn này, nhưng là thanh ghi chuẩn công khai trong datasheet IST8310,
 * được dùng để kiểm tra sự hiện diện của chip và soft-reset.
 ******************************************************************************
 */

#ifndef __IST8310_H
#define __IST8310_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* Thay bằng "stm32f4xx_hal.h" (hoặc dòng STM32 tương ứng) nếu project không có main.h chung */

/* ==================== Địa chỉ I2C ==================== */
#define IST8310_I2C_ADDR            (0x0E << 1)   /* 0x1C, địa chỉ 8-bit dùng cho HAL_I2C_Mem_xxx */

/* ==================== Bản đồ thanh ghi ==================== */
#define IST8310_REG_WAI              0x00   /* Who Am I */
#define IST8310_WAI_VALUE            0x10

#define IST8310_REG_STAT1            0x02   /* Bit0 = DRDY */
#define IST8310_STAT1_DRDY           0x01

#define IST8310_REG_DATA_XL          0x03   /* 6 byte: XL,XH,YL,YH,ZL,ZH */

#define IST8310_REG_CNTL1            0x0A
#define IST8310_CNTL1_SINGLE_MEAS    0x01

#define IST8310_REG_CNTL2            0x0B
#define IST8310_CNTL2_SRST           0x01   /* Soft reset */

#define IST8310_REG_AVGCNTL          0x41
#define IST8310_AVGCNTL_16X          0x24   /* Theo manual: Write 24h vào 0x41h */

#define IST8310_REG_PDCNTL           0x42
#define IST8310_PDCNTL_PULSE         0xC0   /* Theo manual: Write C0h vào 0x42h */

#define IST8310_REG_CROSS_AXIS_START 0x9C   /* 12 byte: Y11..Y33, mỗi giá trị 2 byte (Low, High), 2's complement */

/* Hệ số quy đổi Yab -> Xab theo manual (mục Cross-Axis Compensation) */
#define IST8310_CROSS_AXIS_M         (3.0f / 20.0f)

/* ==================== Mã lỗi trả về ==================== */
#define IST8310_OK                   0x00
#define IST8310_ERROR                0x01
#define IST8310_ERROR_WAI            0x02
#define IST8310_ERROR_TIMEOUT        0x03
#define IST8310_BUSY                 0x04   /* Đang đo, chưa có dữ liệu mới - gọi lại ở vòng lặp sau */

/* ==================== Kiểu dữ liệu ==================== */
typedef struct
{
    int16_t raw_x;      /* Dữ liệu thô XYZ đọc từ 0x03~0x08 */
    int16_t raw_y;
    int16_t raw_z;

    float   mag_x;      /* Dữ liệu sau khi bù cross-axis, đơn vị mili-Gauss (mGauss) */
    float   mag_y;
    float   mag_z;
} IST8310_Data_t;

/* ==================== API ==================== */

/**
 * @brief  Khởi tạo IST8310: kiểm tra WHO_AM_I, soft-reset, cấu hình AVGCNTL/PDCNTL,
 *         đọc và tính sẵn ma trận bù cross-axis từ thanh ghi 0x9C~0xAD.
 * @param  hi2c    Con trỏ tới I2C handle đã init (CubeMX)
 * @param  magData Con trỏ struct dữ liệu, sẽ được init về 0
 * @retval IST8310_OK nếu thành công, khác 0 nếu lỗi (xem các mã IST8310_ERROR_x)
 */
uint8_t IST8310_Init(I2C_HandleTypeDef *hi2c, IST8310_Data_t *magData);

/**
 * @brief  Thực hiện đo NON-BLOCKING (không dùng HAL_Delay) - dùng state machine nội bộ.
 *         PHẢI gọi hàm này liên tục mỗi vòng lặp (loop) để state machine tiến triển:
 *           - Lần gọi đầu tiên (state IDLE): gửi lệnh bắt đầu đo, trả về IST8310_BUSY ngay
 *             (không chờ tại đây).
 *           - Các lần gọi tiếp theo trong lúc chưa đủ 6ms: trả về IST8310_BUSY, hầu như
 *             không tốn thời gian (chỉ so sánh tick, không có I2C nào cả).
 *           - Khi đã đủ thời gian đo tối thiểu: đọc dữ liệu, áp dụng bù cross-axis,
 *             trả về IST8310_OK, rồi tự động quay lại IDLE để bắt đầu lần đo kế tiếp.
 *         Tốc độ cập nhật dữ liệu thực tế ~166Hz (đúng ODR datasheet cho chế độ 16x avg),
 *         không phụ thuộc tốc độ vòng lặp gọi hàm (có thể gọi ở 500Hz, 1kHz... đều an toàn).
 * @param  hi2c    Con trỏ tới I2C handle
 * @param  magData Con trỏ struct để nhận kết quả (raw_x/y/z và mag_x/y/z), chỉ được
 *                  cập nhật khi hàm trả về IST8310_OK.
 * @retval IST8310_OK    Có dữ liệu mới, magData đã được cập nhật
 * @retval IST8310_BUSY  Đang đo, magData giữ nguyên giá trị cũ, gọi lại ở vòng lặp sau
 * @retval khác 0/4       Lỗi (xem các mã IST8310_ERROR_x)
 */
uint8_t IST8310_Read(I2C_HandleTypeDef *hi2c, IST8310_Data_t *magData);

#ifdef __cplusplus
}
#endif

#endif /* __IST8310_H */
