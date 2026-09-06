/**
 * @file fc_flight_modes.c
 * @brief Flight Modes & Position / Navigation Controller Implementation
 */

#include "fc_flight_modes.h"
#include <stdlib.h>

PIDController_t PID_Alt_Pos;
PIDController_t PID_Alt_Vel;
PIDController_t PID_Pos_X;
PIDController_t PID_Pos_Y;
PIDController_t PID_Vel_X;
PIDController_t PID_Vel_Y;

static float kp_alt_pos = 1.0f, ki_alt_pos = 0.0f, kd_alt_pos = 0.0f;
static float kp_alt_vel = 0.8f, ki_alt_vel = 0.0f, kd_alt_vel = 0.003f;

static float kp_xy_pos = 0.8f, ki_xy_pos = 0.0f, kd_xy_pos = 0.0f;
static float kp_xy_vel = 1.5f, ki_xy_vel = 0.0f, kd_xy_vel = 0.2f;

void FlightModes_Init(void) {
    PID_Init(&PID_Alt_Pos, kp_alt_pos, ki_alt_pos, kd_alt_pos, 0.2f);
    PID_Init(&PID_Alt_Vel, kp_alt_vel, ki_alt_vel, kd_alt_vel, 0.2f);
    PID_SetIntegralLimits(&PID_Alt_Vel, -40.0f, 40.0f);

    PID_Init(&PID_Pos_X, kp_xy_pos, ki_xy_pos, kd_xy_pos, 0.2f);
    PID_Init(&PID_Pos_Y, kp_xy_pos, ki_xy_pos, kd_xy_pos, 0.2f);

    PID_Init(&PID_Vel_X, kp_xy_vel, ki_xy_vel, kd_xy_vel, 0.2f);
    PID_SetIntegralLimits(&PID_Vel_X, -6.0f, 6.0f);

    PID_Init(&PID_Vel_Y, kp_xy_vel, ki_xy_vel, kd_xy_vel, 0.2f);
    PID_SetIntegralLimits(&PID_Vel_Y, -6.0f, 6.0f);
}

void Run_XY_Controller(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    // 1. Position PID tracking Carrot setpoint
    float err_x = veh->target_x - veh->est_x;
    float err_y = veh->target_y - veh->est_y;

    veh->target_vx = PID_Calculate(&PID_Pos_X, err_x, dt);
    veh->target_vy = PID_Calculate(&PID_Pos_Y, err_y, dt);

    veh->target_vx = constrain_f(veh->target_vx, -MAX_NAV_SPEED - 1.0f, MAX_NAV_SPEED + 1.0f);
    veh->target_vy = constrain_f(veh->target_vy, -MAX_NAV_SPEED - 1.0f, MAX_NAV_SPEED + 1.0f);

    // 2. Velocity PID tracking
    float err_vx = veh->target_vx - veh->est_vx;
    float err_vy = veh->target_vy - veh->est_vy;

    float out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt);
    float out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt);

    // 3. Rotation Earth -> Body
    float cy = cosf(veh->yaw * DEG_TO_RAD);
    float sy = sinf(veh->yaw * DEG_TO_RAD);

    float force_body_x =  out_angle_earth_x * cy + out_angle_earth_y * sy;
    float force_body_y = -out_angle_earth_x * sy + out_angle_earth_y * cy;

    // 4. Constrain tilt angle
    force_body_x = constrain_f(force_body_x, -20.0f, 20.0f);
    force_body_y = constrain_f(force_body_y, -20.0f, 20.0f);

    // 5. Direct mapping to Drone axes (Front = +Pitch, Right = +Roll)
    veh->target_pitch = force_body_x;
    veh->target_roll  = force_body_y;
}

void Mode_AltHold_Update(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    static bool alt_init = false;

    if (veh->alt_hold_active) {
        if (!alt_init) {
            alt_init = true;
            veh->target_alt = veh->est_alt;
            PID_Reset(&PID_Alt_Pos);
            PID_Reset(&PID_Alt_Vel);
        }

        float alt_err = veh->target_alt - veh->est_alt;
        veh->target_vz = PID_Calculate(&PID_Alt_Pos, alt_err, dt);
        veh->target_vz = constrain_f(veh->target_vz, -MAX_TARGET_VZ, MAX_TARGET_VZ);
    } else {
        alt_init = false;
        veh->target_vz = 0.0f;
        PID_Reset(&PID_Alt_Pos);
    }
}

void Mode_GPS_PositionHold_Update(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    uint8_t gps_hold_sw = veh->rc_raw.nut2;

    // Require switch ON, AltHold ON, and GPS Home locked
    if (veh->rc_raw.nut1 && gps_hold_sw && veh->gps_home_set) {
        if (!veh->pos_hold_active) {
            veh->target_x = veh->est_x;
            veh->target_y = veh->est_y;
            veh->current_wp_index = 0;

            if (Mission_IsReady() && Mission_GetCount() > 0) {
                veh->mission_running = true;
            } else {
                veh->mission_running = false;
            }

            PID_Reset(&PID_Pos_X);
            PID_Reset(&PID_Pos_Y);
            PID_Reset(&PID_Vel_X);
            PID_Reset(&PID_Vel_Y);
            veh->pos_hold_active = true;
        }

        if (veh->mission_running) {
            Waypoint_t current_wp = Mission_GetWaypoint(veh->current_wp_index);

            float wp_lat_err = (float)current_wp.lat - (float)veh->home_lat;
            float wp_lon_err = (float)current_wp.lon - (float)veh->home_lon;
            float wp_target_x = wp_lat_err * 111320.0f;
            float wp_target_y = wp_lon_err * 111320.0f * cosf((float)veh->home_lat * DEG_TO_RAD);

            // Carrot chasing (S-Curve)
            float dx_carrot = wp_target_x - veh->target_x;
            float dy_carrot = wp_target_y - veh->target_y;
            float dist_carrot_to_wp = sqrtf(dx_carrot * dx_carrot + dy_carrot * dy_carrot);

            if (dist_carrot_to_wp > 0.05f) {
                float dir_x = dx_carrot / dist_carrot_to_wp;
                float dir_y = dy_carrot / dist_carrot_to_wp;
                float current_speed = MAX_NAV_SPEED;

                if (dist_carrot_to_wp < 2.0f) {
                    current_speed = dist_carrot_to_wp * 2.5f;
                    if (current_speed < 0.2f) current_speed = 0.2f;
                }

                veh->target_x += dir_x * current_speed * dt;
                veh->target_y += dir_y * current_speed * dt;
            }

            float dx_drone = wp_target_x - veh->est_x;
            float dy_drone = wp_target_y - veh->est_y;
            float dist_to_wp = sqrtf(dx_drone * dx_drone + dy_drone * dy_drone);

            if (dist_to_wp < WAYPOINT_RADIUS) {
                if (veh->current_wp_index < Mission_GetCount() - 1) {
                    veh->current_wp_index++;
                }
            }
        }

        Run_XY_Controller(veh, dt);
        veh->telemetry.target_x = veh->target_roll;
        veh->telemetry.target_y = veh->target_pitch;
    } else {
        if (veh->pos_hold_active) {
            veh->pos_hold_active = false;
            veh->mission_running = false;
        }
    }
}

void Mode_AutoTakeoff(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    if (veh->rc_raw.nut1 && veh->gps_home_set) {
        if (!veh->takeoff_active) {
            veh->target_x = veh->est_x;
            veh->target_y = veh->est_y;
            veh->target_alt = veh->est_alt;

            PID_Reset(&PID_Pos_X);
            PID_Reset(&PID_Pos_Y);
            PID_Reset(&PID_Vel_X);
            PID_Reset(&PID_Vel_Y);
            PID_Reset(&PID_Alt_Pos);
            PID_Reset(&PID_Alt_Vel);
            veh->takeoff_active = true;
            veh->takeoff_complete = false;
        }

        if (!veh->takeoff_complete) {
            veh->target_alt += TAKEOFF_SPEED * dt;
            if (veh->target_alt >= TAKEOFF_TARGET_ALT) {
                veh->target_alt = TAKEOFF_TARGET_ALT;
                if (fabsf(veh->target_alt - veh->est_alt) < 20.0f) {
                    veh->takeoff_complete = true;
                }
            }
        }

        Run_XY_Controller(veh, dt);

        float err_z = veh->target_alt - veh->est_alt;
        veh->target_vz = constrain_f(PID_Calculate(&PID_Alt_Pos, err_z, dt), -2.0f, 2.0f);

        float err_vz = veh->target_vz - veh->est_vz;
        veh->throttle_hover = (int16_t)(HOVER_THROTTLE_BASE + PID_Calculate(&PID_Alt_Vel, err_vz, dt));
    } else {
        veh->takeoff_active = false;
    }
}

void Mode_AutoLanding(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    if (!veh->landing_active) {
        veh->target_x = veh->est_x;
        veh->target_y = veh->est_y;
        veh->target_alt = veh->est_alt;

        PID_Reset(&PID_Pos_X);
        PID_Reset(&PID_Pos_Y);
        PID_Reset(&PID_Vel_X);
        PID_Reset(&PID_Vel_Y);
        PID_Reset(&PID_Alt_Pos);
        PID_Reset(&PID_Alt_Vel);

        veh->landing_active = true;
        veh->landing_complete = false;
    }

    if (!veh->landing_complete) {
        veh->target_alt -= LANDING_SPEED * dt;

        if (veh->target_alt <= 5.0f || veh->est_vz <= 10.0f) {
            veh->target_alt = 0.0f;
            veh->landing_complete = true;
        }

        Run_XY_Controller(veh, dt);

        float err_z = veh->target_alt - veh->est_alt;
        veh->target_vz = constrain_f(PID_Calculate(&PID_Alt_Pos, err_z, dt), -2.0f, 2.0f);

        float err_vz = veh->target_vz - veh->est_vz;
        veh->throttle_hover = (int16_t)(HOVER_THROTTLE_BASE + PID_Calculate(&PID_Alt_Vel, err_vz, dt));
        veh->throttle = veh->throttle_hover;
    } else {
        veh->throttle = 1000;
        veh->target_vz = 0.0f;
        veh->landing_active = false;
    }
}

void FlightModes_Update(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    // Run Altitude Hold outer loop (250Hz)
    Mode_AltHold_Update(veh, dt);

    // Run GPS Position Hold (250Hz)
    Mode_GPS_PositionHold_Update(veh, dt);
}
