/**
 * @file fc_state.h
 * @brief Central Vehicle State Definition (ArduPilot-inspired data sharing)
 */

#ifndef FC_STATE_H_
#define FC_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ==============================================================================
 * FLIGHT MODES & ENUMS
 * ============================================================================== */
typedef enum {
    MODE_STABILIZE = 0,
    MODE_ALT_HOLD,
    MODE_POS_HOLD,
    MODE_AUTO_TAKEOFF,
    MODE_AUTO_LANDING,
    MODE_MISSION
} FlightMode_t;

typedef enum {
    ALT_STATE_IDLE = 0,
    ALT_STATE_TAKEOFF,
    ALT_STATE_HOLD,
    ALT_STATE_LANDING
} AltHoldState_t;

/* ==============================================================================
 * PACKED STRUCTS FOR RADIO RC & TELEMETRY
 * ============================================================================== */
#pragma pack(push, 1)
typedef struct {
    uint16_t trucX;
    uint16_t trucY;
    uint8_t  trai;
    uint8_t  phai;
    uint8_t  len;
    uint8_t  xuong;
    uint8_t  batquathut;
    uint8_t  nut1;
    uint8_t  nut2;
    uint16_t chinhtocdoquat;
    uint8_t  has_wp;
    int32_t  wp_lat;
    int32_t  wp_lon;
} ControlData;

typedef struct {
    int32_t lat;
    int32_t lon;
    float   x;
    float   y;
    float   alt;
    float   target_x;
    float   target_y;
    float   battery;
} TelemetryData;
#pragma pack(pop)

/* ==============================================================================
 * CENTRAL VEHICLE STATE STRUCT
 * ============================================================================== */
typedef struct {
    // 1. Flight Status & Modes
    FlightMode_t    flight_mode;
    AltHoldState_t  alt_state;
    bool            is_armed;
    bool            failsafe_active;
    bool            alt_hold_active;
    bool            pos_hold_active;
    bool            takeoff_active;
    bool            takeoff_complete;
    bool            landing_active;
    bool            landing_complete;
    bool            mission_running;

    // 2. RC Pilot Inputs
    ControlData     rc_raw;
    int16_t         rc_throttle;
    float           rc_target_roll;
    float           rc_target_pitch;
    uint32_t        last_rc_received_tick;
    uint32_t        last_failsafe_throttle_tick;

    // 3. Sensor Measurements (Body Frame)
    float           ax, ay, az;
    float           gx, gy, gz;
    float           ax_offset, ay_offset, az_offset;
    float           gx_offset, gy_offset, gz_offset;
    float           compass_heading;
    float           baro_alt;
    float           baro_alt_offset;
    float           battery_voltage;

    // 4. Attitude Estimation (Euler Angles - degrees)
    float           roll;
    float           pitch;
    float           yaw;
    float           yaw_filt;
    bool            yaw_hold_init;

    // 5. Position & Velocity Estimation (Earth Frame)
    float           ax_earth, ay_earth;
    float           ax_bias, ay_bias;
    float           est_x, est_y;       // meters
    float           est_vx, est_vy;     // m/s
    float           est_alt;            // cm
    float           est_vz;             // cm/s
    float           acc_z_filt;

    // GPS local coordinates & Home
    double          home_lat;
    double          home_lon;
    bool            gps_home_set;
    float           gps_x, gps_y;
    float           gps_vx, gps_vy;
    uint16_t        current_wp_index;

    // 6. Flight Setpoints
    float           target_roll;
    float           target_pitch;
    float           target_yaw;
    float           target_rate_roll;
    float           target_rate_pitch;
    float           target_rate_yaw;
    float           target_alt;         // cm
    float           target_vz;          // cm/s
    float           target_x;           // meters
    float           target_y;           // meters
    float           target_vx;          // m/s
    float           target_vy;          // m/s
    int16_t         throttle;           // Base pilot / controller throttle (1000 - 2000)
    int16_t         throttle_hover;

    // 7. Controller PID Outputs
    float           pid_p;
    float           pid_r;
    float           pid_y;
    float           pid_alt;
    float           pid_alt_vel_out;

    // 8. Motor PWM Outputs (1000 - 2000 us)
    int16_t         motor_pwm[4];

    // 9. Telemetry Outgoing Data
    TelemetryData   telemetry;
} VehicleState_t;

extern VehicleState_t g_veh;

/* ==============================================================================
 * MATH HELPER FUNCTIONS
 * ============================================================================== */
static inline float constrain_f(float data, float min, float max) {
    if (data <= min) return min;
    if (data >= max) return max;
    return data;
}

static inline long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float angle_diff_deg(float target, float current) {
    float diff = target - current;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

#ifdef __cplusplus
}
#endif

#endif /* FC_STATE_H_ */
