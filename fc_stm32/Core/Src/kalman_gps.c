/*
 * kalman_gps.c
 *
 *  Created on: Aug 3, 2026
 *      Author: lethanhtra
 */


#include "kalman_gps.h"

void KalmanGPSAxis_Init(KalmanGPSAxis_t *kf) {
    kf->pos = 0.0f;
    kf->vel = 0.0f;
    kf->bias = 0.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = 0.0f;
        }
    }

    // Khởi tạo độ tin cậy ban đầu (P)
    // Giá trị này thể hiện mức độ "không chắc chắn" lúc vừa bật máy
    kf->P[0][0] = 1.0f;  // Vị trí (m^2)
    kf->P[1][1] = 0.5f;  // Vận tốc ((m/s)^2)
    kf->P[2][2] = 0.5f;  // Bias ((m/s^2)^2) - Để nhỏ vì bias ban đầu thường không lớn
}

void KalmanGPSAxis_Predict(KalmanGPSAxis_t *kf, float accel, float dt) {
    // 1. Tính gia tốc thực tế (đã trừ Bias ước lượng được)
    float acc_corrected = accel - kf->bias;

    // 2. Cập nhật State: pos, vel.
    // Lưu ý: bias được giả định là hằng số hoặc thay đổi rất chậm, nên bias không đổi trong bước Predict.
    kf->pos += kf->vel * dt + 0.5f * acc_corrected * dt * dt;
    kf->vel += acc_corrected * dt;

    // 3. Xây dựng Ma trận Jacobian (F) của mô hình động học
    float c0 = -0.5f * dt * dt; // Đạo hàm của pos theo bias
    float c1 = -dt;             // Đạo hàm của vel theo bias
    const float F[3][3] = {
        {1.0f, dt,   c0},
        {0.0f, 1.0f, c1},
        {0.0f, 0.0f, 1.0f}
    };

    // 4. Tính P_pred = F * P * F^T
    float FP[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }

    float Ppred[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Ppred[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                Ppred[i][j] += FP[i][k] * F[j][k]; // F^T tương đương đổi chỉ số F[j][k]
            }
        }
    }

    // 5. Thêm Ma trận Nhiễu Quá trình (Q)
    float var_acc = 0.01f;        // Phương sai nhiễu đo gia tốc (m/s^2)^2 - Tune thực tế
    float var_bias = 0.0001f;    // Tốc độ trôi của bias (Random Walk) - Phải RẤT NHỎ

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    Ppred[0][0] += 0.25f * dt4 * var_acc;
    Ppred[0][1] += 0.5f  * dt3 * var_acc;
    Ppred[1][0] += 0.5f  * dt3 * var_acc;
    Ppred[1][1] += dt2 * var_acc;
    Ppred[2][2] += var_bias * dt;

    // 6. Cập nhật lại P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = Ppred[i][j];
        }
    }
}

void KalmanGPSAxis_UpdatePos(KalmanGPSAxis_t *kf, float pos_meas, float R) {
    // Với GPS, ta chỉ đo được vị trí -> Ma trận quan sát H = [1, 0, 0]

    // 1. Tính Innovation (y): Độ chênh lệch giữa GPS đo được và Vị trí dự đoán
    float y = pos_meas - kf->pos;

    // 2. Tính S = H * P * H^T + R
    // Do H = [1, 0, 0], S triệt tiêu chỉ còn lại phần tử P[0][0] + R
    float S = kf->P[0][0] + R;
    if (S < 1e-6f) S = 1e-6f; // Chống chia cho 0

    // 3. Tính Kalman Gain (K = P * H^T / S)
    float K[3];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    K[2] = kf->P[2][0] / S;

    // 4. Hiệu chỉnh State (Bí quyết nằm ở K[2] sẽ tự động kéo bias về chuẩn)
    kf->pos  += K[0] * y;
    kf->vel  += K[1] * y;
    kf->bias += K[2] * y;

    // 5. Cập nhật ma trận hiệp phương sai P = (I - K * H) * P
    // Tối ưu hóa ma trận: Do H = [1, 0, 0], dòng code dưới đây là dạng rút gọn toán học của ma trận 3x3
    float Pnew[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Pnew[i][j] = kf->P[i][j] - K[i] * kf->P[0][j];
        }
    }

    // 6. Lưu lại P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = Pnew[i][j];
        }
    }
}
