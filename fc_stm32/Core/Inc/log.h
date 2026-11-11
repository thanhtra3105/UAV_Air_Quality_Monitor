/*
 * log.h
 *
 *  Author: Antigravity - RAM Flight Blackbox Logger (Zero DT Jitter)
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CẤU HÌNH BỘ NHỚ RAM BLACKBOX:
 * 3000 mẫu * 108 bytes = 324,000 bytes (~316 KB RAM, chuẩn ~320KB RAM theo yêu cầu).
 * Tần số ghi: 50 Hz (20ms/mẫu) -> Bay ghi liên tục 60 giây.
 * STM32H562 có 640 KB RAM nên 320 KB RAM hoàn toàn an toàn và thoải mái.
 */
#define MAX_RAM_LOG_SAMPLES     3000

/* Ngưỡng ga bắt đầu ghi log vào RAM: >= 1400 */
#define LOG_THROTTLE_START      1300

/* Ngưỡng ga xác định đã hạ cánh: < 1030 */
#define LOG_THROTTLE_LANDED     1030

/* Cấu trúc dữ liệu ghi log benchmark (108 bytes / mẫu) */
typedef struct {
    uint32_t time_ms;

    // Góc Roll (Target vs Raw vs Complementary vs Kalman)
    float target_roll;
    float roll_raw;
    float roll_cf;
    float roll_kf;

    // Góc Pitch (Target vs Raw vs Complementary vs Kalman)
    float target_pitch;
    float pitch_raw;
    float pitch_cf;
    float pitch_kf;

    // Độ cao & Vận tốc dọc (cm & cm/s) & Gia tốc Z (Đối sánh 2D vs 4D)
    float alt_raw_tof;
    float alt_raw_baro;
    float alt_kf_2d;     // Kalman 2D (không có bias)
    float alt_kf;        // Kalman 4D (có bias acc & baro)
    float vz_raw;
    float vz_kf_2d;      // Kalman 2D (không có bias)
    float vz_kf;         // Kalman 4D (có bias)
    float az_raw;
    float acc_bias;      // Ước lượng Accel bias của Kalman 4D (cm/s^2)
    float baro_bias;     // Ước lượng Baro bias của Kalman 4D (cm)

    // Vị trí ngang (m)
    float pos_x_raw;
    float pos_x_kf;
    float pos_y_raw;
    float pos_y_kf;

    // Vận tốc ngang (m/s)
    float vel_x_raw;
    float vel_x_kf;
    float vel_y_raw;
    float vel_y_kf;
} FlightLogData_t;

/**
 * @brief  Khởi tạo hệ thống Log RAM Blackbox
 */
uint8_t Log_Init(void);

/**
 * @brief  Ghi mẫu trực tiếp vào RAM (Chỉ tốn 0.05 us, 0us trễ loop bay)
 *         Chỉ bắt đầu ghi khi throttle >= 1400.
 *         Khi mảng RAM đầy sẽ tự động ngưng nhận, không làm tràn RAM.
 */
void Log_Write_Custom(const FlightLogData_t *data);

/**
 * @brief  Kiểm tra điều kiện hạ cánh:
 *         Nếu đã từng bay (throttle >= 1400) và nay ga kéo về < 1030 thì
 *         tự động xả toàn bộ dữ liệu từ RAM xuống thẻ MicroSD.
 */
void Log_Check_Auto_Save(void);

/**
 * @brief  Hàm xả RAM xuống thẻ nhớ SD thủ công nếu muốn chủ động gọi.
 */
void Log_Save_RAM_To_SD(void);

/**
 * @brief  Lấy số lượng mẫu hiện tại trong RAM
 */
uint16_t Log_GetSampleCount(void);

/**
 * @brief  Kiểm tra hệ thống có đang bận ghi file xuống SD hay không
 */
bool Log_IsSaving(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOG_H_ */
