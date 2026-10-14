/*
 * pos_hold.c
 *
 *  Created on: Oct 13, 2026
 *      Author: lethanhtra
 */

#include "pos_hold.h"
#include "flight_control.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ================= EXTERNAL SENSORS & CONTROLS ================= */
extern MTF01_t mtf_data;
extern uint8_t mtf01_updated;
extern GPS_Data_t gps;

/* ================= STATE & ESTIMATION VARIABLES ================= */
float ax_earth = 0.0f;
float ay_earth = 0.0f;
float ax_bias = 0.0f;
float ay_bias = 0.0f;

float est_vx = 0.0f;
float est_vy = 0.0f;
float est_x = 0.0f;
float est_y = 0.0f;

float target_x = 0.0f;
float target_y = 0.0f;
float target_vx = 0.0f;
float target_vy = 0.0f;

uint8_t pos_hold_active = 0;

KalmanAxis_t kf_x;
KalmanAxis_t kf_y;

double home_lat = 0.0f;
double home_lon = 0.0f;
uint8_t gps_home_set = 0;

KalmanFilter1D_t kf_North; // X Axis (North)
KalmanFilter1D_t kf_East;  // Y Axis (East)

uint16_t current_wp_index = 0;
uint8_t mission_running = 0;
float lat_err = 0.0f, lon_err = 0.0f;
float pos_N = 0.0f, pos_E = 0.0f;

uint8_t takeoff_active = 0;
uint8_t takeoff_complete = 0;
uint8_t landing_active = 0;
uint8_t landing_complete = 0;

float target_pitch_body = 0.0f;
float target_roll_body = 0.0f;

/* ================= INITIALIZATION ================= */
void PosHold_Init(void) {
	KalmanAxis_Init(&kf_x);
	KalmanAxis_Init(&kf_y);
	KalmanGPS_Init(&kf_North, 0.0f, 0.0f, 0.1f, 0.001f);
	KalmanGPS_Init(&kf_East, 0.0f, 0.0f, 0.1f, 0.001f);
}

/* ================= MTF-01 OPTICAL FLOW ESTIMATION (KALMAN) ================= */
void Estimate_Position_Kalman(float dt_val) {
	/* 1. Rotation Body -> Earth */
	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax + (cy * sp * sr - sy * cr) * ay
			+ (cy * sp * cr + sy * sr) * az;

	ay_earth = sy * cp * ax + (sy * sp * sr + cy * cr) * ay
			+ (sy * sp * cr - cy * sr) * az;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	/* 2. Optical Flow -> Earth */
	float body_vx = -mtf_data.vy; // Tiến/Lùi
	float body_vy = mtf_data.vx;  // Trái/Phải

	float flow_vx = body_vx * cy - body_vy * sy;
	float flow_vy = body_vx * sy + body_vy * cy;

	/* 3. Kalman PREDICT (Dựa vào IMU) */
//	float ax_input = ax_earth - ax_bias;
//	float ay_input = ay_earth - ay_bias;
	KalmanAxis_Predict(&kf_x, ax_earth, dt_val);
	KalmanAxis_Predict(&kf_y, ay_earth, dt_val);

	/* 4. Kalman UPDATE (Dựa vào Optical Flow) */
	if (mtf01_updated) {
		if (mtf_data.flow_quality > 60 && mtf_data.distance >= 100) {
			float err_vx = flow_vx - kf_x.vel;
			float err_vy = flow_vy - kf_y.vel;
			ax_bias += KI_FLOW * err_vx * dt_val;
			ay_bias += KI_FLOW * err_vy * dt_val;
			KalmanAxis_UpdateVel(&kf_x, flow_vx, 0.05);
			KalmanAxis_UpdateVel(&kf_y, flow_vy, 0.05);
		}
	}

	/* 5. Cập nhật biến State cho Position Hold */
	est_x = kf_x.pos;
	est_vx = kf_x.vel;

	est_y = kf_y.pos;
	est_vy = kf_y.vel;
}

/* ================= MTF-01 ESTIMATION (LEGACY LPF) ================= */
void Estimate_Position(float dt_val) {
	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax + (cy * sp * sr - sy * cr) * ay
			+ (cy * sp * cr + sy * sr) * az;

	ay_earth = sy * cp * ax + (sy * sp * sr + cy * cr) * ay
			+ (sy * sp * cr - cy * sr) * az;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	float body_vx = -mtf_data.vy;
	float body_vy = mtf_data.vx;

	float flow_vx = body_vx * cy - body_vy * sy;
	float flow_vy = body_vx * sy + body_vy * cy;

	static float flow_vx_lp = 0.0f;
	static float flow_vy_lp = 0.0f;

	flow_vx_lp += FLOW_ALPHA * (flow_vx - flow_vx_lp);
	flow_vy_lp += FLOW_ALPHA * (flow_vy - flow_vy_lp);

	float ax_correct = ax_earth - ax_bias;
	float ay_correct = ay_earth - ay_bias;

	est_vx += ax_correct * dt_val;
	est_vy += ay_correct * dt_val;

	if (mtf_data.flow_quality > 50 && mtf_data.distance >= 100) {
		float err_vx = flow_vx_lp - est_vx;
		float err_vy = flow_vy_lp - est_vy;

		ax_bias += KI_FLOW * err_vx * dt_val;
		ay_bias += KI_FLOW * err_vy * dt_val;

		est_vx += KP_FLOW * err_vx;
		est_vy += KP_FLOW * err_vy;
	}

	est_x += est_vx * dt_val;
	est_y += est_vy * dt_val;
}
float out_angle_earth_x = 0;
float out_angle_earth_y = 0;
/* ================= MTF-01 POSITION HOLD ================= */
void MTF_PositionHold(float dt_val) {
	uint8_t pos_hold_sw = rxData.nut2;

	if (pos_hold_sw && alt_hold) {
		if (!pos_hold_active) {
			target_x = est_x;
			target_y = est_y;
			PID_Reset(&PID_Pos_X);
			PID_Reset(&PID_Pos_Y);
			PID_Reset(&PID_Vel_X);
			PID_Reset(&PID_Vel_Y);
			pos_hold_active = 1;
		}

		// POSITION LOOP
		float err_x = target_x - est_x;
		float err_y = target_y - est_y;
		if (fabsf(err_x) < 0.01f)
			err_x = 0.0f;
		if (fabsf(err_y) < 0.01f)
			err_y = 0.0f;

		target_vx = PID_Calculate(&PID_Pos_X, err_x, dt_val);
		target_vy = PID_Calculate(&PID_Pos_Y, err_y, dt_val);

		target_vx = constrain(target_vx, -MAX_TARGET_VEL_XY, MAX_TARGET_VEL_XY);
		target_vy = constrain(target_vy, -MAX_TARGET_VEL_XY, MAX_TARGET_VEL_XY);

		// VELOCITY LOOP
		float err_vx = target_vx - est_vx;
		float err_vy = target_vy - est_vy;
		if (fabsf(err_vx) < 0.01f)
			err_vx = 0.0f;
		if (fabsf(err_vy) < 0.01f)
			err_vy = 0.0f;

		out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt_val);
		out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt_val);

		// ROTATION Earth -> Body
		float cy = cosf(yaw * DEG_TO_RAD);
		float sy = sinf(yaw * DEG_TO_RAD);

		target_pitch_body = out_angle_earth_x * cy + out_angle_earth_y * sy;
		target_roll_body = -out_angle_earth_x * sy + out_angle_earth_y * cy;

		target_pitch_body = constrain(target_pitch_body, -15.0f, 15.0f);
		target_roll_body = constrain(target_roll_body, -15.0f, 15.0f);

		target_pitch = target_pitch_body;
		target_roll = target_roll_body;
	} else {
		if (pos_hold_active) {
			pos_hold_active = 0;
		}
		PID_Reset(&PID_Pos_X);
		PID_Reset(&PID_Pos_Y);
		PID_Reset(&PID_Vel_X);
		PID_Reset(&PID_Vel_Y);
	}
}

/* ================= GPS ESTIMATION (KALMAN) ================= */
void Estimate_Position_GPS_Kalman(float dt_val) {
	float ax_frd = ax;
	float ay_frd = -ay;
	float az_frd = -az;

	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax_frd + (-cy * sp * sr - sy * cr) * ay_frd
			+ (-cy * sp * cr + sy * sr) * az_frd;

	ay_earth = sy * cp * ax_frd + (-sy * sp * sr + cy * cr) * ay_frd
			+ (-sy * sp * cr - cy * sr) * az_frd;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	KalmanGPS_Predict(&kf_North, ax_earth, dt_val);
	KalmanGPS_Predict(&kf_East, ay_earth, dt_val);

	if (gps.fixType >= 3 && gps.ready == 1) {
		if (!gps_home_set) {
			home_lat = gps.latitude;
			home_lon = gps.longitude;
			gps_home_set = 1;

			kf_North.pos = 0.0f;
			kf_North.vel = 0.0f;
			kf_North.bias = 0.0f;
			kf_East.pos = 0.0f;
			kf_East.vel = 0.0f;
			kf_East.bias = 0.0f;
		} else {
			lat_err = (float) gps.latitude - home_lat;
			lon_err = (float) gps.longitude - home_lon;

			pos_N = lat_err * 111320.0f;
			pos_E = lon_err * 111320.0f * cosf(home_lat * DEG_TO_RAD);

			float vel_N = (float) gps.velN / 1000.0f;
			float vel_E = (float) gps.velE / 1000.0f;

			float r_pos_noise = (float) (gps.hAcc / 1000.0f)
					* (gps.hAcc / 1000.0f);
			float r_vel_noise = (float) (gps.sAcc / 1000.0f)
					* (gps.sAcc / 1000.0f);

			KalmanGPS_Update(&kf_North, pos_N, vel_N, r_pos_noise, r_vel_noise);
			KalmanGPS_Update(&kf_East, pos_E, vel_E, r_pos_noise, r_vel_noise);
		}
		gps.ready = 0;
	}

	est_x = kf_North.pos;
	est_vx = kf_North.vel;
	est_y = kf_East.pos;
	est_vy = kf_East.vel;
}

/* ================= SHARED CARROT CHASING XY CONTROLLER ================= */
void Run_XY_Controller(float dt_val) {
	float err_x = target_x - est_x;
	float err_y = target_y - est_y;

	target_vx = PID_Calculate(&PID_Pos_X, err_x, dt_val);
	target_vy = PID_Calculate(&PID_Pos_Y, err_y, dt_val);

	target_vx = constrain(target_vx, -MAX_NAV_SPEED - 1.0f,
	MAX_NAV_SPEED + 1.0f);
	target_vy = constrain(target_vy, -MAX_NAV_SPEED - 1.0f,
	MAX_NAV_SPEED + 1.0f);

	float err_vx = target_vx - est_vx;
	float err_vy = target_vy - est_vy;

	float out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt_val);
	float out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt_val);

	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);

	float force_body_x = out_angle_earth_x * cy + out_angle_earth_y * sy;
	float force_body_y = -out_angle_earth_x * sy + out_angle_earth_y * cy;

	force_body_x = constrain(force_body_x, -20.0f, 20.0f);
	force_body_y = constrain(force_body_y, -20.0f, 20.0f);

	target_pitch = force_body_x;
	target_roll = force_body_y;
}

/* ================= GPS WAYPOINT POSITION HOLD ================= */
void GPS_PositionHold(float dt_val) {
	uint8_t gps_hold_sw = rxData.nut2;

	if (rxData.nut1 && gps_hold_sw && gps_home_set) {
		if (!pos_hold_active) {
			target_x = est_x;
			target_y = est_y;

			current_wp_index = 0;

			if (Mission_IsReady() && Mission_GetCount() > 0) {
				mission_running = 1;
			} else {
				mission_running = 0;
			}

			PID_Reset(&PID_Pos_X);
			PID_Reset(&PID_Pos_Y);
			PID_Reset(&PID_Vel_X);
			PID_Reset(&PID_Vel_Y);
			pos_hold_active = 1;
		}

		if (mission_running) {
			Waypoint_t current_wp = Mission_GetWaypoint(current_wp_index);

			float wp_lat_err = (float) current_wp.lat - home_lat;
			float wp_lon_err = (float) current_wp.lon - home_lon;
			float wp_target_x = wp_lat_err * 111320.0f;
			float wp_target_y = wp_lon_err * 111320.0f
					* cosf(home_lat * DEG_TO_RAD);

			// S-Curve Carrot Chasing
			float dx_carrot = wp_target_x - target_x;
			float dy_carrot = wp_target_y - target_y;
			float dist_carrot_to_wp = sqrtf(
					dx_carrot * dx_carrot + dy_carrot * dy_carrot);

			if (dist_carrot_to_wp > 0.05f) {
				float dir_x = dx_carrot / dist_carrot_to_wp;
				float dir_y = dy_carrot / dist_carrot_to_wp;
				float current_speed = MAX_NAV_SPEED;

				if (dist_carrot_to_wp < 2.0f) {
					current_speed = dist_carrot_to_wp * 2.5f;
					if (current_speed < 0.2f)
						current_speed = 0.2f;
				}

				target_x += dir_x * current_speed * dt_val;
				target_y += dir_y * current_speed * dt_val;
			}

			float dx_drone = wp_target_x - est_x;
			float dy_drone = wp_target_y - est_y;
			float dist_to_wp = sqrtf(dx_drone * dx_drone + dy_drone * dy_drone);

			if (dist_to_wp < WAYPOINT_RADIUS) {
				if (current_wp_index < Mission_GetCount() - 1) {
					current_wp_index++;
				}
			}
		}
		Run_XY_Controller(dt_val);
		txData.target_x = target_roll;
		txData.target_y = target_pitch;
	} else {
		if (pos_hold_active) {
			pos_hold_active = 0;
			mission_running = 0;
		}
	}
}

/* ================= AUTO TAKEOFF & LANDING ================= */
void Mode_AutoTakeoff(float dt_val) {
	if (rxData.nut1 && gps_home_set) {
		if (!takeoff_active) {
			target_x = est_x;
			target_y = est_y;
			target_alt = kf_4d.altitude;

			PID_Reset(&PID_Pos_X);
			PID_Reset(&PID_Pos_Y);
			PID_Reset(&PID_Vel_X);
			PID_Reset(&PID_Vel_Y);
			PID_Reset(&PID_Alt_Pos);
			PID_Reset(&PID_Alt_Vel);
			takeoff_active = 1;
			takeoff_complete = 0;
		}

		if (!takeoff_complete) {
			target_alt += TAKEOFF_SPEED * dt_val;
			if (target_alt >= TAKEOFF_TARGET_ALT) {
				target_alt = TAKEOFF_TARGET_ALT;
				if (abs((int) (target_alt - kf_4d.altitude)) < 20)
					takeoff_complete = 1;
			}
		}

		Run_XY_Controller(dt_val);
		float err_z = target_alt - kf_4d.altitude;
		target_vz = constrain(PID_Calculate(&PID_Alt_Pos, err_z, dt_val), -2.0f,
				2.0f);

		float err_vz = target_vz - kf_4d.velocity;
		throttle_hover = HOVER_THROTTLE_BASE
				+ PID_Calculate(&PID_Alt_Vel, err_vz, dt_val);
	} else {
		takeoff_active = 0;
	}
}

void Mode_AutoLanding(float dt_val) {
	if (!landing_active) {
		target_x = est_x;
		target_y = est_y;
		target_alt = kf_4d.altitude;

		PID_Reset(&PID_Pos_X);
		PID_Reset(&PID_Pos_Y);
		PID_Reset(&PID_Vel_X);
		PID_Reset(&PID_Vel_Y);
		PID_Reset(&PID_Alt_Pos);
		PID_Reset(&PID_Alt_Vel);

		landing_active = 1;
		landing_complete = 0;
	}

	if (!landing_complete) {
		target_alt -= LANDING_SPEED * dt_val;

		if (target_alt <= 5.0f || kf_4d.velocity <= 10.0f) {
			target_alt = 0.0f;
			landing_complete = 1;
		}

		Run_XY_Controller(dt_val);

		float err_z = target_alt - kf_4d.altitude;
		target_vz = constrain(PID_Calculate(&PID_Alt_Pos, err_z, dt_val), -2.0f,
				2.0f);

		float err_vz = target_vz - kf_4d.velocity;
		throttle_hover = HOVER_THROTTLE_BASE
				+ PID_Calculate(&PID_Alt_Vel, err_vz, dt_val);
		throttle = throttle_hover;
	} else {
		throttle = 1000;
		target_vz = 0;
		landing_active = 0;
	}
}
