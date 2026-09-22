/*
 * kalman.h
 *
 *  Created on: Jun 30, 2026
 *      Author: lethanhtra
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "stm32h5xx_hal.h"
#include "main.h"

extern float KalmanAngleRoll;
extern float KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch;
extern float KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[];  // [0]: Góc, [1]: Sai số
/* Nhieu qua trinh / do cua Kalman filter - TUNE theo thuc te bay */
static const float KF_Q_ACCEL = 0.35f; // do "tin" gia toc IMU (nhieu qua trinh tren accel, don vi (m/s^2)^2 * s ap dung don gian)
static const float KF_R_BASE = 0.05f; // nhieu do co ban cua van toc do tu optical flow (m/s)^2, quality=255

typedef struct {
	float altitude;      // Trạng thái S[0]: Cao độ ước lượng
	float velocity;      // Trạng thái S[1]: Vận tốc dọc ước lượng
	float P[2][2];       // Ma trận hiệp phương sai sai số P
} Kalman2D_t;

// kalman.h — thêm struct mới
typedef struct {
	float altitude;
	float velocity;
	float baro_bias;   // trôi chậm của baro, ước lượng riêng
	float P[3][3];
} Kalman3D_t;

typedef struct {
	float altitude;		// cm
	float velocity;		// cm/s
	float acc_bias;
	float baro_bias;
	float terrain_offset;	// cm — độ cao mặt đất cục bộ so với gốc altitude
							// (dương nếu mặt đất/bụi cỏ nhô cao hơn điểm gốc)
	float P[5][5];			// Giữ tên Kalman4D_t để không phá vỡ code khác
							// đang tham chiếu kiểu này, dù nay là 5-state
} Kalman4D_t;

typedef struct {
	float pos;
	float vel;
	float P[2][2];

} KalmanAxis_t;

void Kalman1D_Compute(float KalmanState, float KalmanUncertainty,
		float KalmanInput, float KalmanMeasurement, float dt);

// (Tùy chọn) Hàm gộp chung cả Predict và Update giống kalman_2d cũ của bạn
void Kalman2D_Compute(Kalman2D_t *kf, float acc_z, float alt_measured,
		float sigma_alt, float dt);
void KalmanAxis_Init(KalmanAxis_t *kf);
void KalmanAxis_Predict(KalmanAxis_t *kf, float accel, float dt);
void KalmanAxis_UpdateVel(KalmanAxis_t *kf, float vel_measure, float R);
void KalmanAxis_UpdatePos(KalmanAxis_t *kf, float pos_meas, float R);
void KalmanAxis_Compute(KalmanAxis_t *kf, float accel, float vel_measure,
		float R, float dt);

void Kalman4D_Init(Kalman4D_t *kf, float initial_alt);
void Kalman4D_Predict(Kalman4D_t *kf, float acc_z, float dt);
void Kalman4D_UpdateMeasure(Kalman4D_t *kf, float alt_measured, float sigma_alt,
		float H[4]);

void Kalman4D_UpdateRange(Kalman4D_t *kf, float range_alt_cm, float acc_z);
void Kalman4D_UpdateBaro(Kalman4D_t *kf, float baro_alt_cm);

void Kalman4D_UpdateHeight(Kalman4D_t *kf, float baro_alt_cm, float range_cm,
		float roll_deg, float pitch_deg, float acc_z_cm_s2, float dt,
		uint8_t baro_valid, uint8_t range_valid);

float Kalman4D_getSigmaRange(float range_alt_cm, float current_estimated_alt_cm, float acc_z);
float Kalman4D_getSigmaBaro(float current_estimated_alt_cm);

#endif /* INC_KALMAN_H_ */
