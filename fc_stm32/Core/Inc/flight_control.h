/*
 * flight_control.h
 *
 *  Created on: Oct 13, 2026
 *      Author: lethanhtra
 */

#ifndef INC_FLIGHT_CONTROL_H_
#define INC_FLIGHT_CONTROL_H_

#include "main.h"
#include "types.h"
#include "pid_controller.h"
#include "kalman.h"
#include "icm20602.h"
#include "dsp310.h"
#include "ist8310.h"
#include "qmc5883.h"

/* ================= CONFIGURATION DEFINES ================= */
#define USE_IST8310
//#define USE_QMC5883

#define TAKEOFF_TARGET_ALT  130.0f  // Độ cao cất cánh mục tiêu (cm)
#define TAKEOFF_SPEED       80.0f  // Tốc độ cất cánh (cm/s)
#define HOVER_THROTTLE_BASE 1650.0f
#define PILOT_TAKE_OFF_ALT  200.0f  // 2 mét (cm)
#define CLIMB_RATE          150.0f  // Tốc độ leo 1.5 m/s
#define DESCEND_RATE        100.0f  // Tốc độ hạ 1.0 m/s
#define DEADZONE_LOW        1500
#define DEADZONE_HIGH       1550

#define MAX_TARGET_ROLL_PITCH 8.0f
#define MAX_TARGET_VZ         80.0f
#define MAX_ALT_OUTPUT        200.0f

/* ================= EXTERN VARIABLES ================= */
extern ControlData rxData;
extern TelemetryData txData;
extern ICM20602_t imu;
extern DSP310_t dsp_sensor;
extern Kalman4D_t kf_4d;

#ifdef USE_QMC5883
extern QMC5883_t mag;
#else
extern IST8310_Data_t ist8310;
#endif

// IMU Raw & Offsets
extern float dt;
extern float gx, gy, gz;
extern float ax, ay, az;
extern float ax_offset, ay_offset, az_offset;
extern float gx_offset, gy_offset, gz_offset;

// Attitude & Heading
extern float roll, pitch, yaw;
extern float target_roll, target_pitch, target_yaw;
extern float target_rate_roll, target_rate_pitch, target_rate_yaw;

// Altitude & Baro
extern float alt_offset;
extern float current_alt, target_alt, target_vz;
extern uint8_t alt_hold, alt_init;

// Throttle & Outputs
extern int throttle;
extern int throttle_hover;
extern int throttle_rx;
extern float pid_p, pid_r, pid_y, pid_alt, pid_alt_vel_out;

// RC State
extern uint32_t timeout_connected, time_throttle;

// PID Controllers
extern PIDController_t PID_Angle;
extern PIDController_t PID_Rate_Pitch, PID_Rate_Roll, PID_Rate_Yaw;
extern PIDController_t PID_Alt, PID_Alt_Pos, PID_Alt_Vel;
extern PIDController_t PID_Pos_X, PID_Pos_Y, PID_Vel_X, PID_Vel_Y;

// PID Gains
extern float kp_angle, ki_angle, kd_angle;
extern float kp_r, ki_r, kd_r;
extern float kp_p, ki_p, kd_p;
extern float kp_y, ki_y, kd_y;
extern float kp_alt, ki_alt, kd_alt;
extern float kp_alt_pos, ki_alt_pos, kd_alt_pos;
extern float kp_alt_vel, ki_alt_vel, kd_alt_vel;
extern float kp_xy_pos, ki_xy_pos, kd_xy_pos;
extern float kp_xy_vel, ki_xy_vel, kd_xy_vel;

/* ================= FUNCTION PROTOTYPES ================= */
void FlightControl_Init(void);
void readIMU(void);
void calibIMU(void);
float readHeading(float roll_deg, float pitch_deg);
float readAltitude(float pressure_hPa);
float DSP_CalibrationAltitude(uint8_t sample);
float angle_diff(float target, float current);
void calculateAngle(float dt);
void calculatePIDAngle(float dt_angle);
void calcualatePIDRate(float dt_rate);
void PID_Task(float dt_pid);
void writeMotor(uint8_t chanel, int16_t us);
void mixer(void);
void RxController(void);
void Failsafe_Task(void);

#endif /* INC_FLIGHT_CONTROL_H_ */
