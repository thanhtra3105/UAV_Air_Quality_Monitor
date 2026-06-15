#pragma once

#include <BasicLinearAlgebra.h>

class AltitudeEstimator {
public:

  float altitude = 0.0f;
  float velocity = 0.0f;

  void begin();
  void update(float dt,
              float acc_z,
              float tof_alt,
              bool tof_valid,
              float baro_alt);

private:

  BLA::Matrix<2, 1> X;

  BLA::Matrix<2, 2> F;
  BLA::Matrix<2, 1> G;

  BLA::Matrix<2, 2> P;
  BLA::Matrix<2, 2> Q;

  BLA::Matrix<2, 2> H;
  BLA::Matrix<2, 2> R;

  BLA::Matrix<2, 2> I;

  BLA::Matrix<2, 2> S;
  BLA::Matrix<2, 2> K;

  BLA::Matrix<2, 1> Z;
};