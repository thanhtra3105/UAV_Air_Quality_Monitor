/*
 * kalman.c
 *
 *  Created on: Jun 30, 2026
 *      Author: lethanhtra
 */

#include "kalman.h"

float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };  // [0]: Góc, [1]: Sai số


void Kalman1D_Compute(float KalmanState, float KalmanUncertainty, float KalmanInput,
		float KalmanMeasurement, float dt) {
	// Dự đoán trạng thái mới dựa trên vận tốc góc (Gyro) và thời gian thực dt
	KalmanState = KalmanState + dt * KalmanInput;
	// Cập nhật sai số dự đoán (4*4 là phương sai Gyro - có thể chỉnh để lọc mượt hơn)
	KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;

	// Tính toán Kalman Gain (3*3 là phương sai Accelerometer)
	float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

	// Cập nhật trạng thái bằng phép đo từ Gia tốc kế
	KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
	// Cập nhật sai số sau khi đã hiệu chỉnh
	KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

	Kalman1DOutput[0] = KalmanState;
	Kalman1DOutput[1] = KalmanUncertainty;
}

void Kalman2D_Init(Kalman2D_t *kf, float initial_alt) {
    kf->altitude = initial_alt;
    kf->velocity = 0.0f;
    // Khởi tạo ma trận P như trong hàm kalman_setup() của bạn
    kf->P[0][0] = 10.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 10.0f;
}

// Bước DỰ ĐOÁN (Predict)
void Kalman2D_Predict(Kalman2D_t *kf, float acc_z, float dt) {
    // 1. Cập nhật trạng thái: S = F*S + G*Acc
    kf->altitude += dt * kf->velocity + 0.5f * dt * dt * acc_z;
    kf->velocity += dt * acc_z;

    // 2. Cập nhật sai số: P = F*P*F^T + Q
    float sigma_acc = 10.0f; // cm/s² (từ code gốc của bạn)
    float var_acc = sigma_acc * sigma_acc;

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    // Tính ma trận Q = G * G^T * var_acc
    float Q00 = 0.25f * dt4 * var_acc;
    float Q01 = 0.5f  * dt3 * var_acc;
    float Q11 = dt2 * var_acc;

    // Lưu tạm P để tính toán không bị đè dữ liệu
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];
    float P10 = kf->P[1][0];
    float P11 = kf->P[1][1];

    kf->P[0][0] = P00 + dt * (P01 + P10) + dt2 * P11 + Q00;
    kf->P[0][1] = P01 + dt * P11 + Q01;
    kf->P[1][0] = kf->P[0][1]; // Ma trận P luôn đối xứng (P10 = P01)
    kf->P[1][1] = P11 + Q11;
}

// Bước CẬP NHẬT (Update)
void Kalman2D_Update(Kalman2D_t *kf, float alt_measured, float sigma_alt) {
    // 1. Tính Kalman Gain: K = P * H^T * (H*P*H^T + R)^-1
    float R = sigma_alt * sigma_alt;
    float L = kf->P[0][0] + R; // L = H*P*H^T + R

    float K0 = kf->P[0][0] / L;
    float K1 = kf->P[1][0] / L;

    // 2. Hiệu chỉnh trạng thái: S = S + K * (M - H*S)
    float y = alt_measured - kf->altitude; // Sai số (Innovation)
    kf->altitude += K0 * y;
    kf->velocity += K1 * y;

    // 3. Cập nhật lại sai số hiệp phương sai: P = (I - K*H) * P
    float P00 = kf->P[0][0];
    float P01 = kf->P[0][1];

    kf->P[0][0] -= K0 * P00;
    kf->P[0][1] -= K0 * P01;
    kf->P[1][0] = kf->P[0][1]; // Đối xứng
    kf->P[1][1] -= K1 * P01;
}

// Hàm gộp (thay thế cho kalman_2d gốc của bạn)
void Kalman2D_Compute(Kalman2D_t *kf, float acc_z, float alt_measured, float sigma_alt, float dt) {
    Kalman2D_Predict(kf, acc_z, dt);
    Kalman2D_Update(kf, alt_measured, sigma_alt);
}



/* ================================================================
 *                    KALMAN FILTER 1 TRUC (pos, vel)
 *  Mo hinh:  x = [pos; vel],  u = accel (control input)
 *  F = [[1, dt], [0, 1]] ,  B = [[0.5*dt^2], [dt]]
 *  Do (measurement) chi la van toc: H = [0, 1]
 * ================================================================ */
void KalmanAxis_Init(KalmanAxis_t *kf)
{
    kf->pos = 0.0f;
    kf->vel = 0.0f;
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
}

void KalmanAxis_Predict(KalmanAxis_t *kf, float accel, float dt)
{
    // ----- Predict trang thai -----
    kf->pos = kf->pos + kf->vel * dt + 0.5f * accel * dt * dt;
    kf->vel = kf->vel + accel * dt;

    // ----- Predict hiep phuong sai: P = F*P*F^T + Q -----
    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];

    float newP00 = P00 + dt * (P01 + P10) + dt * dt * P11;
    float newP01 = P01 + dt * P11;
    float newP10 = P10 + dt * P11;
    float newP11 = P11;

    // Nhieu qua trinh don gian hoa (them truc tiep, du dung cho embedded thuc te)
    newP00 += KF_Q_ACCEL * dt * dt * dt * 0.25f;
    newP11 += KF_Q_ACCEL * dt;

    kf->P[0][0] = newP00; kf->P[0][1] = newP01;
    kf->P[1][0] = newP10; kf->P[1][1] = newP11;
}

void KalmanAxis_UpdateVel(KalmanAxis_t *kf, float measured_vel, float R)
{
    // Residual (innovation)
    float y = measured_vel - kf->vel;

    // S = H*P*H^T + R = P11 + R
    float S = kf->P[1][1] + R;
    if (S < 1e-6f) S = 1e-6f;

    // Kalman gain K = P*H^T / S
    float K0 = kf->P[0][1] / S;
    float K1 = kf->P[1][1] / S;

    kf->pos += K0 * y;
    kf->vel += K1 * y;

    float P00 = kf->P[0][0], P01 = kf->P[0][1];
    float P10 = kf->P[1][0], P11 = kf->P[1][1];

    kf->P[0][0] = P00 - K0 * P10;
    kf->P[0][1] = P01 - K0 * P11;
    kf->P[1][0] = P10 - K1 * P10;
    kf->P[1][1] = P11 - K1 * P11;
}

void KalmanAxis_Compute(KalmanAxis_t *kf,
                        float accel,
                        float vel_measure,
                        float R,
                        float dt)
{
    KalmanAxis_Predict(kf, accel, dt);
    KalmanAxis_UpdateVel(kf, vel_measure, R);
}
