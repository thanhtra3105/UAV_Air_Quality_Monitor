/*
 * kalman_gps.h
 *
 *  Created on: Aug 3, 2026
 *      Author: lethanhtra
 */

#ifndef KALMAN_GPS_H
#define KALMAN_GPS_H

#include <stdint.h>

// Cấu trúc Lọc Kalman 3 trạng thái cho 1 trục tọa độ (X hoặc Y)
typedef struct {
    float pos;      // Trạng thái 0: Vị trí ước lượng (m)
    float vel;      // Trạng thái 1: Vận tốc ước lượng (m/s)
    float bias;     // Trạng thái 2: Nhiễu gia tốc kế (m/s^2)
    float P[3][3];  // Ma trận hiệp phương sai sai số
} KalmanGPSAxis_t;

// Khởi tạo bộ lọc
void KalmanGPSAxis_Init(KalmanGPSAxis_t *kf);

// Bước DỰ ĐOÁN (chạy ở vòng lặp nhanh 500Hz cùng IMU)
void KalmanGPSAxis_Predict(KalmanGPSAxis_t *kf, float accel, float dt);

// Bước CẬP NHẬT (chạy ở vòng lặp chậm ~5Hz khi có dữ liệu GPS mới)
void KalmanGPSAxis_UpdatePos(KalmanGPSAxis_t *kf, float pos_meas, float R);

#endif /* KALMAN_GPS_H */
