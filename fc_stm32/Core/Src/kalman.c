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

void Kalman1D_Compute(float KalmanState, float KalmanUncertainty,
		float KalmanInput, float KalmanMeasurement, float dt) {
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

/* ================================================================
 *                    KALMAN FILTER 1 TRUC (pos, vel)
 *  Mo hinh:  x = [pos; vel],  u = accel (control input)
 *  F = [[1, dt], [0, 1]] ,  B = [[0.5*dt^2], [dt]]
 *  Do (measurement) chi la van toc: H = [0, 1]
 * ================================================================ */
void KalmanAxis_Init(KalmanAxis_t *kf) {
	kf->pos = 0.0f;
	kf->vel = 0.0f;
	kf->P[0][0] = 1.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 1.0f;
}

void KalmanAxis_Predict(KalmanAxis_t *kf, float accel, float dt) {
	// ----- Predict trang thai -----
	kf->pos = kf->pos + kf->vel * dt + 0.5f * accel * dt * dt;
	kf->vel = kf->vel + accel * dt;

	// ----- Predict hiep phuong sai: P = F*P*F^T + Q -----
	float P00 = kf->P[0][0], P01 = kf->P[0][1];
	float P10 = kf->P[1][0], P11 = kf->P[1][1];

	float newP00 = P00 + dt * (P01 + P10) + dt * dt * P11;
	float newP01 = P01 + dt * P11;
	float newP10 = newP01; 	// matran doi xung nen cho bang nhau luon
	float newP11 = P11;

	newP00 += KF_Q_ACCEL * dt * dt * dt * dt * 0.25f;
	newP01 += KF_Q_ACCEL * dt * dt * dt * 0.5f;
	newP10 = newP01;
	newP11 += KF_Q_ACCEL * dt * dt;

	kf->P[0][0] = newP00;
	kf->P[0][1] = newP01;
	kf->P[1][0] = newP10;
	kf->P[1][1] = newP11;
}

void KalmanAxis_UpdateVel(KalmanAxis_t *kf, float measured_vel, float R) {
	float P00 = kf->P[0][0];
	float P01 = kf->P[0][1];
	float P11 = kf->P[1][1];
	// 1. Tinh Kalman Gain: K[Kp,Kv]
	float S = kf->P[1][1] + R;
	if (S < 1e-6f)
		S = 1e-6f;
	float Kp = kf->P[0][1] / S;
	float Kv = kf->P[1][1] / S;

	//2. Cap nhat ma tran voi do luong:
	float innovation = measured_vel - kf->vel;
	kf->pos += Kp * innovation;
	kf->vel += Kv * innovation;

	//3. Update ma tran hiep phuong sai (covariance)
	float newP00 = P00 - Kp * P01;
	float newP01 = P01 - Kp * P11;
	float newP11 = P11 - Kv * P11;

	kf->P[0][0] = newP00;
	kf->P[0][1] = newP01;
	kf->P[1][0] = newP01;
	kf->P[1][1] = newP11;
}

void KalmanAxis_UpdatePos(KalmanAxis_t *kf, float pos_meas, float R) {
	// Residual (innovation) - Sai số vị trí
	float y = pos_meas - kf->pos;

	// S = H*P*H^T + R = P00 + R (Vì ma trận H = [1, 0])
	float S = kf->P[0][0] + R;
	if (S < 1e-6f)
		S = 1e-6f;

	// Kalman gain K = P*H^T / S
	float K0 = kf->P[0][0] / S;
	float K1 = kf->P[1][0] / S;

	// Cập nhật lại State
	kf->pos += K0 * y;
	kf->vel += K1 * y;

	// Cập nhật ma trận hiệp phương sai P = (I - K*H) * P
	float P00 = kf->P[0][0], P01 = kf->P[0][1];
	float P10 = kf->P[1][0], P11 = kf->P[1][1];

	kf->P[0][0] = P00 - K0 * P00;
	kf->P[0][1] = P01 - K0 * P01;
	kf->P[1][0] = P10 - K1 * P00;
	kf->P[1][1] = P11 - K1 * P01;
}

void KalmanAxis_Compute(KalmanAxis_t *kf, float accel, float vel_measure,
		float R, float dt) {
	KalmanAxis_Predict(kf, accel, dt);
	KalmanAxis_UpdateVel(kf, vel_measure, R);
}

/*
 * kalman_fusion.c
 * Sensor Fusion: IMU (Acc_Z) + Baro (DPS310) + Rangefinder (MTF-01P)
 */
/*
 * kalman.c
 * Sensor Fusion: IMU (acc_z) + DPS310 Barometer + Rangefinder (MTF01-P)
 */

void Kalman4D_Init(Kalman4D_t *kf, float initial_alt) {
	kf->altitude = initial_alt;
	kf->velocity = 0.0f;
	kf->acc_bias = 0.0f;
	kf->baro_bias = 0.0f;

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			kf->P[i][j] = 0.0f;

	kf->P[0][0] = 10.0f; // uncertainty ban đầu của altitude
	kf->P[1][1] = 10.0f; // velocity
	kf->P[2][2] = 1.0f;  // acc_bias (cm/s²) — chỉnh theo thực nghiệm
	kf->P[3][3] = 5.0f;  // baro_bias (cm)
}

void Kalman4D_Predict(Kalman4D_t *kf, float acc_z, float dt) {
	float acc_corrected = acc_z - kf->acc_bias;

	// ----- Predict trạng thái -----
	kf->altitude += dt * kf->velocity + 0.5f * dt * dt * acc_corrected;
	kf->velocity += dt * acc_corrected;
	// acc_bias, baro_bias giữ nguyên, chỉ trôi qua Q

	// ----- F (Jacobian) -----
	float a = dt;
	float c0 = -0.5f * dt * dt; // ảnh hưởng acc_bias lên altitude
	float c1 = -dt;             // ảnh hưởng acc_bias lên velocity

	const float F[4][4] = { { 1, a, c0, 0 }, { 0, 1, c1, 0 }, { 0, 0, 1, 0 }, {
			0, 0, 0, 1 } };

	// FP = F * P
	float FP[4][4];
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			FP[i][j] = 0.0f;
			for (int k = 0; k < 4; k++)
				FP[i][j] += F[i][k] * kf->P[k][j];
		}

	// Ppred = FP * F^T
	float Ppred[4][4];
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			Ppred[i][j] = 0.0f;
			for (int k = 0; k < 4; k++)
				Ppred[i][j] += FP[i][k] * F[j][k];
		}

	// ----- Cộng nhiễu quá trình Q -----
	float var_acc = 10.0f * 10.0f; // sigma_acc, giữ nguyên đơn vị như code cũ (cm/s²)
	float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt2 * dt2;

	Ppred[0][0] += 0.25f * dt4 * var_acc;
	Ppred[0][1] += 0.5f * dt3 * var_acc;
	Ppred[1][0] += 0.5f * dt3 * var_acc;
	Ppred[1][1] += dt2 * var_acc;

	Ppred[2][2] += 1e-5f * dt; // acc_bias trôi RẤT chậm — accel bias thực tế đổi theo nhiệt độ, không đổi theo rung
	Ppred[3][3] += 0.01f * dt; // baro_bias, giữ như bản cũ

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			kf->P[i][j] = Ppred[i][j];
}

void Kalman4D_UpdateMeasure(Kalman4D_t *kf, float alt_measured, float sigma_alt,
		float H[4]) {
	float R = sigma_alt * sigma_alt;

	// PHt = P * H^T
	float PHt[4];
	for (int i = 0; i < 4; i++) {
		PHt[i] = 0.0f;
		for (int j = 0; j < 4; j++)
			PHt[i] += kf->P[i][j] * H[j];
	}

	// S = H*P*H^T + R
	float S = R;
	for (int i = 0; i < 4; i++)
		S += H[i] * PHt[i];
	if (S < 1e-6f)
		S = 1e-6f;

	// K = PHt / S
	float K[4];
	for (int i = 0; i < 4; i++)
		K[i] = PHt[i] / S;

	// ----- SỬA LỖI INNOVATION Ở ĐÂY -----
	// Lấy trạng thái hiện tại nhân với ma trận H để ra giá trị dự đoán
	float h_x = kf->altitude * H[0] + kf->velocity * H[1] + kf->acc_bias * H[2]
			+ kf->baro_bias * H[3];

	float y = alt_measured - h_x; // Sai số giữa thực tế và dự đoán

	kf->altitude += K[0] * y;
	kf->velocity += K[1] * y;
	kf->acc_bias += K[2] * y;
	kf->baro_bias += K[3] * y;

	// P = (I - K*H) * P
	float M[4][4];
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			M[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];

	float Pnew[4][4];
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++) {
			Pnew[i][j] = 0.0f;
			for (int k = 0; k < 4; k++)
				Pnew[i][j] += M[i][k] * kf->P[k][j];
		}

	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			kf->P[i][j] = Pnew[i][j];
}

// Ngưỡng độ cao chuyển giao (cm)
#define TRANSITION_ALT 150.0f

float Kalman4D_getSigmaBaro(float current_estimated_alt_cm) {
	float base_sigma = 20.0f; // Nhiễu tiêu chuẩn của Baro khi bay cao (cm)

	// Nếu bay dưới 1.5m, ground effect từ cánh quạt làm nhiễu Baro -> Tăng R
	if (current_estimated_alt_cm < TRANSITION_ALT) {
		float factor = (TRANSITION_ALT - current_estimated_alt_cm)
				/ TRANSITION_ALT;
		if (factor < 0.0f)
			factor = 0.0f;
		// Ở mặt đất (0m) sigma = 230cm, lên đến 1.5m sigma giảm dần về 30cm
		base_sigma += factor * 200.0f;
	}
	return base_sigma;
}

float Kalman4D_getSigmaRange(float range_alt_cm, float current_estimated_alt_cm,
		float acc_z) {
	float base_sigma = 2.0f; // Rangefinder đo rất chính xác ở tầm thấp (5cm)

	// 1. Phân kì tầm xa: Lên quá cao (2m) thì Rangefinder không còn là nguồn ưu tiên
	if (range_alt_cm > 200.0f || current_estimated_alt_cm > 200.0f) {
		return 10000.0f; // Phạt R cực lớn để Kalman gần như lờ đi Rangefinder
	}

	// 2. Chống nhiễu địa hình (bụi cỏ, hố trũng)
	float diff = fabsf(range_alt_cm - current_estimated_alt_cm);
	float acc_z_abs = fabsf(acc_z);

	// Nếu Rangefinder báo nhảy độ cao > 20cm NHƯNG gia tốc Z không lớn (< 2m/s^2)
	// Chứng tỏ drone không thực sự giật lên/xuống mà do địa hình bên dưới thay đổi.
	if (diff > 20.0f && acc_z_abs < 200.0f) {
		// Tăng nhiễu tỉ lệ thuận với độ lệch để hệ thống chuyển qua bám vào Baro/Acc
		base_sigma += diff * 10.0f;
	}

	return base_sigma;
}

// Cập nhật Baro: Tính sigma dựa trên độ cao ước lượng hiện tại của KF
void Kalman4D_UpdateBaro(Kalman4D_t *kf, float baro_alt_cm) {
	float H[4] = { 1, 0, 0, 1 }; // z = altitude + baro_bias
	float sigma_baro = Kalman4D_getSigmaBaro(kf->altitude);
	Kalman4D_UpdateMeasure(kf, baro_alt_cm, sigma_baro, H);
}

float sigma_range;
// Cập nhật Rangefinder: Tính sigma dựa trên biến động của range và acc_z
void Kalman4D_UpdateRange(Kalman4D_t *kf, float range_alt_cm, float acc_z) {
	float H[4] = { 1, 0, 0, 0 }; // Rangefinder đo độ cao thật, không có baro_bias

	// Yêu cầu bạn truyền acc_z vào hàm này (lưu ý: acc_z phải là gia tốc đã trừ đi trọng lực 1G)
	sigma_range = Kalman4D_getSigmaRange(range_alt_cm, kf->altitude, acc_z);
	Kalman4D_UpdateMeasure(kf, range_alt_cm, sigma_range, H);
}

