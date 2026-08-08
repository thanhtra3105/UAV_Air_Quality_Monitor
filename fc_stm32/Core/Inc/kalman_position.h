/*
 * kalman_position.h
 *
 *  Created on: Jul 19, 2026
 *      Author: lethanhtra
 */

#ifndef KALMAN_POSITION_H
#define KALMAN_POSITION_H

#include <stdint.h>

typedef struct {
    float dt;           // Chu kỳ trích mẫu (0.004s)
    float p;            // Trạng thái ước lượng: Vị trí (m)
    float v;            // Trạng thái ước lượng: Vận tốc (m/s)

    // Ma trận hiệp phương sai sai số P (2x2 phẳng)
    float P00, P01;
    float P10, P11;

    // Tham số nhiễu hệ thống (Q) và nhiễu đo lường (R)
    float Q_accel;      // Nhiễu từ gia tốc kế IMU
    float R_gps;        // Nhiễu từ sai số định vị GPS
} KalmanPos_t;

void KalmanPos_Init(KalmanPos_t *kf, float dt, float q_accel, float r_gps);
void KalmanPos_Predict(KalmanPos_t *kf, float acc_earth_frame);
void KalmanPos_UpdateGPS(KalmanPos_t *kf, float gps_pos);

#endif
