/*
 * kalman.h
 *
 *  Created on: Jun 30, 2026
 *      Author: lethanhtra
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "stm32h5xx_hal.h"

extern float KalmanAngleRoll;
extern float KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch;
extern float KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[];  // [0]: Góc, [1]: Sai số
/* Nhieu qua trinh / do cua Kalman filter - TUNE theo thuc te bay */
static const float KF_Q_ACCEL = 0.35f;   // do "tin" gia toc IMU (nhieu qua trinh tren accel, don vi (m/s^2)^2 * s ap dung don gian)
static const float KF_R_BASE  = 0.05f;   // nhieu do co ban cua van toc do tu optical flow (m/s)^2, quality=255

typedef struct {
	float altitude;      // Trạng thái S[0]: Cao độ ước lượng
	float velocity;      // Trạng thái S[1]: Vận tốc dọc ước lượng
	float P[2][2];       // Ma trận hiệp phương sai sai số P
} Kalman2D_t;

typedef struct {
	float pos;
	float vel;
	float P[2][2];

} KalmanAxis_t;

void Kalman1D_Compute(float KalmanState, float KalmanUncertainty,
		float KalmanInput, float KalmanMeasurement, float dt);

// --- Các hàm cho Kalman 2D (Độ cao & Vận tốc) ---
void Kalman2D_Init(Kalman2D_t *kf, float initial_alt);
void Kalman2D_Predict(Kalman2D_t *kf, float acc_z, float dt);
void Kalman2D_Update(Kalman2D_t *kf, float alt_measured, float sigma_alt);

// (Tùy chọn) Hàm gộp chung cả Predict và Update giống kalman_2d cũ của bạn
void Kalman2D_Compute(Kalman2D_t *kf, float acc_z, float alt_measured,
		float sigma_alt, float dt);
void KalmanAxis_Init(KalmanAxis_t *kf);
void KalmanAxis_Predict(KalmanAxis_t *kf, float accel, float dt);
void KalmanAxis_UpdateVel(KalmanAxis_t *kf, float vel_measure, float R);
void KalmanAxis_Compute(KalmanAxis_t *kf, float accel, float vel_measure,
		float R, float dt);
#endif /* INC_KALMAN_H_ */
