/*
 * kalman_position.c
 *
 *  Created on: Jul 19, 2026
 *      Author: lethanhtra
 */


#include "kalman_position.h"

void KalmanPos_Init(KalmanPos_t *kf, float dt, float q_accel, float r_gps) {
    kf->dt = dt;
    kf->p = 0.0f;
    kf->v = 0.0f;

    // Khởi tạo ma trận P bằng ma trận đơn vị nhân với trọng số lớn
    kf->P00 = 1.0f; kf->P01 = 0.0f;
    kf->P10 = 0.0f; kf->P11 = 1.0f;

    kf->Q_accel = q_accel;
    kf->R_gps = r_gps;
}

// Bước dự đoán (Predict) - Gọi liên tục ở tần số 250Hz
void KalmanPos_Predict(KalmanPos_t *kf, float acc_earth_frame) {
    float dt = kf->dt;
    float dt2 = dt * dt;

    // 1. Cập nhật trạng thái bằng hàm truyền động học x = A*x + B*u
    kf->p = kf->p + (kf->v * dt) + (0.5f * acc_earth_frame * dt2);
    kf->v = kf->v + (acc_earth_frame * dt);

    // 2. Tính toán ma trận Q nhanh (Nhiễu hệ thống tích hợp qua dt)
    float Q00 = 0.25f * dt2 * dt2 * kf->Q_accel;
    float Q01 = 0.5f * dt2 * dt * kf->Q_accel;
    float Q10 = Q01;
    float Q11 = dt2 * kf->Q_accel;

    // 3. Cập nhật ma trận hiệp phương sai P = A*P*A^T + Q
    float P00_new = kf->P00 + dt*(kf->P10 + kf->P01) + dt2*kf->P11 + Q00;
    float P01_new = kf->P01 + dt*kf->P11 + Q01;
    float P10_new = kf->P10 + dt*kf->P11 + Q10;
    float P11_new = kf->P11 + Q11;

    kf->P00 = P00_new; kf->P01 = P01_new;
    kf->P10 = P10_new; kf->P11 = P11_new;
}

// Bước cập nhật (Update) - Chỉ gọi khi cờ dữ liệu mới từ GPS bật lên (ví dụ: 5Hz hoặc 10Hz)
void KalmanPos_UpdateGPS(KalmanPos_t *kf, float gps_pos) {
    // 1. Tính toán Innovation: y = z - H*x (H = [1, 0] nên H*x chính là vị trí p)
    float y = gps_pos - kf->p;

    // 2. Tính toán S = H*P*H^T + R
    float S = kf->P00 + kf->R_gps;

    // 3. Tính toán Kalman Gain: K = P * H^T * S^-1
    float K0 = kf->P00 / S;
    float K1 = kf->P10 / S;

    // 4. Cập nhật trạng thái tối ưu: x = x + K*y
    kf->p += K0 * y;
    kf->v += K1 * y;

    // 5. Cập nhật hiệp phương sai sai số: P = (I - K*H)*P
    float P00_new = (1.0f - K0) * kf->P00;
    float P01_new = (1.0f - K0) * kf->P01;
    float P10_new = kf->P10 - K1 * kf->P00;
    float P11_new = kf->P11 - K1 * kf->P01;

    kf->P00 = P00_new; kf->P01 = P01_new;
    kf->P10 = P10_new; kf->P11 = P11_new;
}
