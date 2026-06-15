#include "altitude_estimator.h"

using namespace BLA;

void AltitudeEstimator::begin() {
  X = { 0, 0 };

  P = { 10, 0,
        0, 10 };

  I = { 1, 0,
        0, 1 };
}

void AltitudeEstimator::update(float dt,
                               float acc_z,
                               float tof_alt,
                               bool tof_valid,
                               float baro_alt) {
  F = { 1, dt,
        0, 1 };

  G = { 0.5f * dt * dt,
        dt };

  float sigma_acc = 150.0f;

  Q = G * ~G * sigma_acc * sigma_acc;

  //-----------------------------------
  // Predict
  //-----------------------------------

  X = F * X + G * acc_z;

  P = F * P * ~F + Q;

  //-----------------------------------
  // Adaptive measurement noise
  //-----------------------------------

  float Rtof;

  if (!tof_valid) {
    Rtof = 1000.0f;
  } else if (tof_alt < 8.0f) {
    Rtof = 2.0f * 2.0f;
  } else {
    float k = (tof_alt - 8.0f) / 2.0f;

    if (k > 1.0f) k = 1.0f;

    float sigma = 0.03f + k * 2.0f;

    Rtof = sigma * sigma;
  }

  float Rbaro = 50.0f * 50.0f;

  //-----------------------------------
  // Update
  //-----------------------------------

  H = {
    1, 0,
    1, 0
  };

  R = {
    Rtof, 0,
    0, Rbaro
  };

  Z = {
    tof_alt,
    baro_alt
  };

  S = H * P * ~H + R;

  K = P * ~H * Inverse(S);

  X = X + K * (Z - H * X);

  P = (I - K * H) * P;

  altitude = X(0);
  velocity = X(1);
}