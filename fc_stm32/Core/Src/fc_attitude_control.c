/**
 * @file fc_attitude_control.c
 * @brief Cascaded Attitude & Angular Rate PID Controller Implementation
 */

#include "fc_attitude_control.h"

PIDController_t PID_Angle;
PIDController_t PID_Rate_Pitch;
PIDController_t PID_Rate_Roll;
PIDController_t PID_Rate_Yaw;

// Gains matching original main.c
static float kp_angle = 6.5f, ki_angle = 0.0f, kd_angle = 0.0f;
static float kp_r = 0.85f, ki_r = 1.0f, kd_r = 0.003f;
static float kp_p = 0.85f, ki_p = 1.0f, kd_p = 0.003f;
static float kp_y = 0.85f, ki_y = 0.0f, kd_y = 0.003f;

// Extern PID_Alt_Vel from flight modes
extern PIDController_t PID_Alt_Vel;

void AttitudeControl_Init(void) {
    PID_Init(&PID_Angle, kp_angle, ki_angle, kd_angle, 0.2f);

    PID_Init(&PID_Rate_Pitch, kp_p, ki_p, kd_p, 0.2f);
    PID_SetIntegralLimits(&PID_Rate_Pitch, -60.0f, 60.0f);

    PID_Init(&PID_Rate_Roll, kp_r, ki_r, kd_r, 0.2f);
    PID_SetIntegralLimits(&PID_Rate_Roll, -60.0f, 60.0f);

    PID_Init(&PID_Rate_Yaw, kp_y, ki_y, kd_y, 0.2f);
}

void AttitudeControl_ResetAll(void) {
    PID_Reset(&PID_Angle);
    PID_Reset(&PID_Rate_Pitch);
    PID_Reset(&PID_Rate_Roll);
    PID_Reset(&PID_Rate_Yaw);
}

void AttitudeControl_AngleLoop(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    // Outer angle loop: Error (deg) -> Target angular rate (deg/s)
    float pitch_err = veh->target_pitch - veh->pitch;
    veh->target_rate_pitch = PID_Calculate(&PID_Angle, pitch_err, dt);

    float roll_err = veh->target_roll - veh->roll;
    veh->target_rate_roll = PID_Calculate(&PID_Angle, roll_err, dt);

    float yaw_err = angle_diff_deg(veh->target_yaw, veh->yaw);
    veh->target_rate_yaw = PID_Calculate(&PID_Angle, yaw_err, dt);

    // Rate constraints (-200 to +200 deg/s)
    veh->target_rate_pitch = constrain_f(veh->target_rate_pitch, -200.0f, 200.0f);
    veh->target_rate_roll  = constrain_f(veh->target_rate_roll,  -200.0f, 200.0f);
    veh->target_rate_yaw   = constrain_f(veh->target_rate_yaw,   -200.0f, 200.0f);
}

void AttitudeControl_RateLoop(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    // Reset integral terms at low throttle to prevent ground windup
    if (veh->throttle < 1300) {
        AttitudeControl_ResetAll();
    }

    // Inner rate loop: Angular rate error (deg/s) -> Motor torque commands
    float pitch_rate_err = veh->target_rate_pitch - veh->gy;
    veh->pid_p = PID_Calculate(&PID_Rate_Pitch, pitch_rate_err, dt);

    float roll_rate_err = veh->target_rate_roll - veh->gx;
    veh->pid_r = PID_Calculate(&PID_Rate_Roll, roll_rate_err, dt);

    float yaw_rate_err = veh->target_rate_yaw - veh->gz;
    veh->pid_y = PID_Calculate(&PID_Rate_Yaw, yaw_rate_err, dt);

    // Altitude rate / velocity inner loop
    if (veh->alt_hold_active) {
        float vz_err = veh->target_vz - veh->est_vz;
        veh->pid_alt = PID_Calculate(&PID_Alt_Vel, vz_err, dt);
    } else {
        veh->pid_alt = 0.0f;
        PID_Reset(&PID_Alt_Vel);
    }

    // Constraints on PID outputs
    veh->pid_r   = constrain_f(veh->pid_r,   -300.0f, 300.0f);
    veh->pid_p   = constrain_f(veh->pid_p,   -300.0f, 300.0f);
    veh->pid_y   = constrain_f(veh->pid_y,   -300.0f, 300.0f);
    veh->pid_alt = constrain_f(veh->pid_alt, -MAX_ALT_OUTPUT, MAX_ALT_OUTPUT);
}
