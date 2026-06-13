#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;

extern float acc_z_inertial;
extern float current_alt;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };  // [0]: Góc, [1]: Sai số

extern float dt;

BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;

void kalman_setup() {
  F = { 1, 0.004, 0, 1 };
  G = { 0.5 * 0.004 * 0.004, 0.004 };
  H = { 1, 0 };
  I = { 1, 0, 0, 1 };
  Q = G * ~G * 10.0f * 10.0f;
  R = { 30 * 30 };
  P = { 10, 0, 0, 10 };
  S = { 0, 0 };
}


void kalman_2d(float acc_z_inertial, float current_alt) {
  Acc = { acc_z_inertial };
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Inverse(L);
  M = { current_alt };
  S = S + K * (M - H * S);
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  P = (I - K * H) * P;
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
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