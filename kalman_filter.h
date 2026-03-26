#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;

extern float acc_z_inertial;
extern float current_alt;

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
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  S = S + K * (M - H * S);
  P = (I - K * H) * P;
}