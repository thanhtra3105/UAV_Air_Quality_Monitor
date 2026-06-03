#pragma once
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

using namespace BLA;

// ─── Đầu ra Kalman 2D (altitude + velocity) ───────────────
float AltitudeKalman       = 0.0f;
float VelocityVerticalKalman = 0.0f;

// ─── Biến Kalman 1D — dùng làm output tạm thời ─────────────
// QUAN TRỌNG: Chỉ được gọi từ TaskAngleControl (không gọi đồng thời từ 2 task)
float Kalman1DOutput[2] = { 0.0f, 0.0f };  // [0]=angle, [1]=uncertainty

// ─── Kalman 2D matrices ─────────────────────────────────────
static BLA::Matrix<2,2> F_k, P_k, Q_k, I_k;
static BLA::Matrix<2,1> G_k, S_k, K_k;
static BLA::Matrix<1,2> H_k;
static BLA::Matrix<1,1> Acc_k, R_k, L_k, M_k;

void kalman_setup() {
  F_k = { 1, 0.002f, 0, 1 };
  G_k = { 0.5f * 0.002f * 0.002f, 0.002f };
  H_k = { 1, 0 };
  I_k = { 1, 0, 0, 1 };
  Q_k = G_k * ~G_k * 100.0f;   // process noise (10^2)
  R_k = { 900.0f };             // measurement noise (30^2)
  P_k = { 10, 0, 0, 10 };
  S_k = { 0, 0 };
}

// Kalman 2D: altitude + vertical velocity
// Gọi từ TaskAngleControl (100Hz) — không gọi đồng thời ở nơi khác
void kalman_2d(float acc_z_inertial_m_s2, float current_alt_cm, float dt, float measurement_noise_cm2) {
  // State dùng đơn vị cm và cm/s, nên gia tốc phải đổi từ m/s^2 sang cm/s^2
  float acc_z_cm_s2 = acc_z_inertial_m_s2 * 100.0f;

  F_k = { 1, dt,
          0, 1  };
  G_k = { 0.5f * dt * dt,
          dt };

  // Q càng lớn thì filter càng tin IMU/gia tốc thay đổi nhanh. Tune sau khi bay thử.
  Q_k = G_k * ~G_k * 2500.0f;
  R_k = { measurement_noise_cm2 };

  Acc_k = { acc_z_cm_s2 };
  S_k   = F_k * S_k + G_k * Acc_k;
  P_k   = F_k * P_k * ~F_k + Q_k;

  L_k   = H_k * P_k * ~H_k + R_k;
  K_k   = P_k * ~H_k * Inverse(L_k);

  M_k   = { current_alt_cm };
  S_k   = S_k + K_k * (M_k - H_k * S_k);

  AltitudeKalman = S_k(0, 0);
  VelocityVerticalKalman = S_k(1, 0);

  P_k = (I_k - K_k * H_k) * P_k;
}

// Kalman 1D: lọc góc pitch/roll
// Kết quả ghi vào Kalman1DOutput[0] (angle) và [1] (uncertainty)
// Gọi từ TaskAngleControl — không thread-safe nếu gọi song song
void kalman_1d(float KalmanState, float KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement, float dt) {
  // Predict
  KalmanState       += dt * KalmanInput;
  KalmanUncertainty += dt * dt * 16.0f;  // gyro variance = 4^2

  // Update
  float KalmanGain   = KalmanUncertainty / (KalmanUncertainty + 9.0f);  // accel variance = 3^2
  KalmanState       += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty  = (1.0f - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}