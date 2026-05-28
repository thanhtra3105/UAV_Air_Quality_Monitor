// types.h
#pragma once

struct FlightState {
  float pitch = 0, roll = 0, yaw = 0;
  float pitch_acc = 0, roll_acc = 0;
  float target_pitch = 0;
  float target_roll  = 0;
  float target_yaw   = 0;
  float target_alt   = 0;
  float target_rate_pitch = 0;
  float target_rate_roll  = 0;
  float target_rate_yaw   = 0;
  int   throttle = 1000;
  float pid_p = 0, pid_r = 0, pid_yaw = 0;
  float pid_vel   = 0;
  float acc_z_inertial = 0;
  bool  alt_hold  = false;
  bool  pos_hold  = false;
  uint8_t flight_mode = 1;
  float KalmanUncertaintyAngleRoll  = 4.0f;
  float KalmanUncertaintyAnglePitch = 4.0f;
  float AltitudeKalman = 0;           // ← thêm để handleData dùng được
  float VelocityVerticalKalman = 0;   // ← thêm để handleData dùng được
};

struct PidGains {
  float kp_r = 0.80f, ki_r = 0.0001f, kd_r = 0.002f;
  float kp_p = 0.80f, ki_p = 0.0001f, kd_p = 0.002f;
  float kp_y = 0.80f, ki_y = 0.0001f, kd_y = 0.002f;
  float kp_angle = 6.5f, ki_angle = 0.0f, kd_angle = 0.0f;
  float kp_vel_z = 3.5f, ki_vel_z = 0.0015f, kd_vel_z = 0.01f;
  float danh_lai = 4.0f;
};