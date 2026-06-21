#pragma once
#include <stdint.h>
#include <stdbool.h>

extern uint8_t flight_mode;
extern float gps_pitch_pid_adjust, gps_roll_pid_adjust;
extern uint8_t waypoint_set;
// ── API ────────────────────────────────────────────────────
void gpsSetup();
void readGPS();
bool checkGPSQuality();
void gps_debug();


// #ifndef POSHOLD_H
// #define POSHOLD_H

// extern bool pos_hold_enable;

// extern float hold_x;
// extern float hold_y;

// extern float target_pitch_gps;
// extern float target_roll_gps;

// void posHoldInit();
// void posHoldUpdate(float dt);

// #endif
