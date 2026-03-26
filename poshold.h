#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── GPS data (khai báo ở poshold.cpp, dùng extern ở main) ──
extern float current_lat;
extern float current_lon;
extern float current_speed_ms;
extern float gpsHeading;
extern float hdop;
extern uint8_t satellites;

// ── Setpoint vị trí giữ ────────────────────────────────────
extern float hold_lat;
extern float hold_lon;

// ── API ────────────────────────────────────────────────────
void gpsSetup();
void readGPS();
bool checkGPSQuality();
void poshold_predict_imu();   // gọi mỗi lần đọc IMU (~200Hz)
void poshold_update_gps();    // gọi bên trong readGPS() khi có fix mới
void calculatePosHold();      // gọi trong loop()