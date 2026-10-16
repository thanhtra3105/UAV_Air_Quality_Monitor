/*
 * pos_hold.h
 *
 *  Created on: Oct 13, 2026
 *      Author: lethanhtra
 */

#ifndef INC_POS_HOLD_H_
#define INC_POS_HOLD_H_

#include "main.h"
#include "types.h"
#include "kalman.h"
#include "kalman_gps.h"
#include "mtf01.h"
#include "gps.h"
#include "mission.h"

/* ================= CONFIGURATION DEFINES ================= */
#define FLOW_ALPHA          0.2f
#define KP_FLOW             0.25f
#define KI_FLOW             0.02f
#define KP_POS              0.10f
#define OPTICAL_FLOW_R      0.5f     // Phương sai nhiễu đo vận tốc Flow

#define MAX_TARGET_VEL_XY   1.0f     // m/s
#define GPS_POS_R           2.5f
#define KI_GPS_BIAS         0.02f
#define MAX_ACCEL_BIAS      1.0f

#define WAYPOINT_RADIUS     2.0f     // Bán kính xác nhận đến điểm (m)
#define MAX_NAV_SPEED       5.0f     // Tốc độ di chuyển tối đa (m/s)
#define LANDING_SPEED       20.0f    // Tốc độ hạ cánh (cm/s)

/* ================= EXTERN STATE VARIABLES ================= */
// Gia tốc & Bias trong Earth Frame
extern float ax_earth, ay_earth;
extern float ax_bias, ay_bias;

// Vận tốc & Vị trí ước lượng (Earth Frame)
extern float est_vx, est_vy;
extern float est_x, est_y;

// Điểm neo mục tiêu (Target)
extern float target_x, target_y;
extern float target_vx, target_vy;
extern uint8_t pos_hold_active;

// Kalman Filter Optical Flow
extern KalmanAxis_t kf_x, kf_y;

// GPS Home & Navigation
extern double home_lat, home_lon;
extern uint8_t gps_home_set;
extern KalmanFilter1D_t kf_North, kf_East;
extern uint16_t current_wp_index;
extern uint8_t mission_running;
extern float lat_err, lon_err;
extern float pos_N, pos_E;

// Auto Takeoff & Landing
extern uint8_t takeoff_active, takeoff_complete;
extern uint8_t landing_active, landing_complete;
extern float target_pitch_body, target_roll_body;

/* ================= FUNCTION PROTOTYPES ================= */
void PosHold_Init(void);
void Estimate_Position_Kalman(float dt);
void Estimate_Position(float dt);
void MTF_PositionHold(float dt);
void Estimate_Position_GPS_Kalman(float dt);
void Run_XY_Controller(float dt);
void GPS_PositionHold(float dt);
void Mode_AutoTakeoff(float dt);
void Mode_AutoLanding(float dt);
void Takeoff_Init(void);
void FlightMode_Execute(float dt_val);
void Landing_Init(void);

#endif /* INC_POS_HOLD_H_ */
