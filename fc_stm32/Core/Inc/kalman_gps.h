/*
 * kalman_gps.h
 *
 *  Created on: Aug 3, 2026
 *      Author: lethanhtra
 */

#ifndef KALMAN_GPS_H
#define KALMAN_GPS_H

//#include <stdint.h>
//
//// Cấu trúc Lọc Kalman 3 trạng thái cho 1 trục tọa độ (X hoặc Y)
//typedef struct {
//    float pos;      // Trạng thái 0: Vị trí ước lượng (m)
//    float vel;      // Trạng thái 1: Vận tốc ước lượng (m/s)
//    float bias;     // Trạng thái 2: Nhiễu gia tốc kế (m/s^2)
//    float P[3][3];  // Ma trận hiệp phương sai sai số
//} KalmanGPSAxis_t;
//
//// Khởi tạo bộ lọc
//void KalmanGPSAxis_Init(KalmanGPSAxis_t *kf);
//
//// Bước DỰ ĐOÁN (chạy ở vòng lặp nhanh 500Hz cùng IMU)
//void KalmanGPSAxis_Predict(KalmanGPSAxis_t *kf, float accel, float dt);
//
//// Bước CẬP NHẬT (chạy ở vòng lặp chậm ~5Hz khi có dữ liệu GPS mới)
//void KalmanGPSAxis_UpdatePos(KalmanGPSAxis_t *kf, float pos_meas, float R);


#include <stdint.h>
#include <math.h>

// Cấu trúc lưu trữ dữ liệu của bộ lọc Kalman 1 Trục (1D)
typedef struct {
    // Trạng thái hệ thống (States)
    float pos;      // Vị trí (m)
    float vel;      // Vận tốc (m/s)
    float bias;     // Sai số gia tốc IMU (m/s^2)

    // Ma trận hiệp phương sai sai số (Covariance Matrix 3x3)
    float P[3][3];

    // Các hằng số nhiễu hệ thống (System Noise - Tuning)
    float Q_accel;  // Nhiễu nhiễu gia tốc (độ rung của drone)
    float Q_bias;   // Tốc độ trôi sai số IMU (cần cài rất nhỏ)
} KalmanFilter1D_t;

/**
 * @brief Khởi tạo bộ lọc Kalman
 * @param kf Con trỏ đến cấu trúc bộ lọc
 * @param init_pos Vị trí ban đầu (thường là 0)
 * @param init_vel Vận tốc ban đầu (thường là 0)
 * @param q_accel Hệ số nhiễu gia tốc (Ví dụ: 0.5f)
 * @param q_bias Hệ số trôi cảm biến (Ví dụ: 0.001f)
 */
void KalmanGPS_Init(KalmanFilter1D_t *kf, float init_pos, float init_vel, float q_accel, float q_bias);

/**
 * @brief Bước 1: Dự đoán trạng thái từ IMU (Chạy ở ngắt tần số cao, vd 100-400Hz)
 * @param kf Con trỏ đến cấu trúc bộ lọc
 * @param accel_earth Gia tốc tuyến tính theo trục Trái Đất (đã loại bỏ trọng lực và xoay hệ tọa độ)
 * @param dt Thời gian delta time (giây) kể từ lần gọi predict trước đó
 */
void KalmanGPS_Predict(KalmanFilter1D_t *kf, float accel_earth, float dt);

/**
 * @brief Bước 2: Cập nhật sửa lỗi từ GPS (Chạy khi nhận được bản tin GPS mới, vd 5-10Hz)
 * @param kf Con trỏ đến cấu trúc bộ lọc
 * @param gps_pos Tọa độ đo được từ GPS (m)
 * @param gps_vel Vận tốc đo được từ Doppler GPS (m/s)
 * @param r_pos Sai số vị trí GPS (gpsPosAccuracy lấy từ bản tin UBX)
 * @param r_vel Sai số vận tốc GPS (gpsSpdAccuracy lấy từ bản tin UBX)
 */
void KalmanGPS_Update(KalmanFilter1D_t *kf, float gps_pos, float gps_vel, float r_pos, float r_vel);

#endif /* KALMAN_GPS_H */
