/*
 * log.h
 *
 *  Author: Antigravity - RAM Flight Blackbox Logger (Zero DT Jitter)
 */

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "stm32h5xx_hal.h"
#include "gps.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CẤU HÌNH BỘ NHỚ RAM BLACKBOX (500Hz Accel + 10Hz GPS Logger):
 * 12,000 mẫu * 32 bytes = 384,000 bytes (~375 KB RAM / 640 KB RAM STM32H562).
 * Tần số ghi: 500 Hz (2ms/mẫu) -> Ghi liên tục 24 giây bay lơ lửng.
 */
#define MAX_RAM_LOG_SAMPLES     12000

/* Ngưỡng ga bắt đầu ghi log vào RAM: >= 1300 */
#define LOG_THROTTLE_START      1300

/* Ngưỡng ga xác định đã hạ cánh: < 1030 */
#define LOG_THROTTLE_LANDED     1030

/* Cấu trúc dữ liệu ghi gia tốc 500Hz + GPS (32 bytes / mẫu) */
typedef struct {
    uint32_t time_us;      // Thời gian vi giây DWT (chính xác tuyệt đối)
    float    ax_earth;     // Gia tốc trục North (m/s^2)
    float    ay_earth;     // Gia tốc trục East (m/s^2)
    float    velN;         // Vận tốc North từ GPS (m/s)
    float    velE;         // Vận tốc East từ GPS (m/s)
    float    hAcc;         // Sai số vị trí GPS công bố (m)
    float    sAcc;         // Sai số tốc độ GPS công bố (m/s)
    uint8_t  gps_updated;  // 1: Mẫu mới từ GPS; 0: Mẫu cũ giữ nguyên
    uint8_t  padding[3];   // Căn lề 32-bit
} AccelLog_t;

/* Cấu trúc FlightLogData_t giữ lại để không xung đột mã cũ */
typedef struct {
    uint32_t time_ms;
    float target_roll, roll_raw, roll_cf, roll_kf;
    float target_pitch, pitch_raw, pitch_cf, pitch_kf;
    float alt_raw_tof, alt_raw_baro, alt_kf_2d, alt_kf;
    float vz_raw, vz_kf_2d, vz_kf, az_raw;
    float acc_bias, baro_bias;
    float ax_earth, ay_earth;
    float pos_x_raw, pos_x_kf, pos_y_raw, pos_y_kf;
    float vel_x_raw, vel_x_kf, vel_y_raw, vel_y_kf;
} FlightLogData_t;

/**
 * @brief  Khởi tạo hệ thống Log RAM Blackbox
 */
uint8_t Log_Init(void);

/**
 * @brief  Ghi toàn bộ thông tin bay (Accel 500Hz + GPS) vào RAM
 */
void Log_Write_Flight(uint32_t time_us, float ax, float ay, const GPS_Data_t *myGPS);

/**
 * @brief  Ghi chi tiết dữ liệu bay vào RAM
 */
void Log_Write_Data(uint32_t time_us, float ax, float ay, float velN, float velE, float hAcc, float sAcc, uint8_t gps_updated);

/**
 * @brief  Ghi gia tốc + tự động lấy GPS vào RAM (Tương thích ngược hàm cũ)
 */
void Log_Write_Accel(uint32_t time_us, float ax, float ay);

/**
 * @brief  Tự động xả RAM xuống thẻ SD khi hạ cánh (throttle < 1030)
 */
void Log_Check_Auto_Save(void);

/**
 * @brief  Xả dữ liệu xuống thẻ MicroSD
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
