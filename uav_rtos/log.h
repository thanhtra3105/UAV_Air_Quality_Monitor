#pragma once
#include "SPIFFS.h"

// ─── Cờ log ─────────────────────────────────────────────────
// log_flag được set bởi TaskCommunication, đọc bởi TaskTelemetry
// Vì chỉ là 1 byte và viết từ 1 nơi, volatile là đủ
extern volatile uint8_t log_flag;

// ─── Các biến cần log — định nghĩa trong uav_rtos.ino ────────
extern float acc_z_inertial;
extern volatile float current_altitude;     // cm (từ BMP280, cập nhật bởi Telemetry)
extern float VelocityVerticalKalman;
extern float pid_vel;
extern float bat_voltage;
extern volatile int throttle;
extern float az;                   // raw az (nếu cần)

// ─── Struct log ──────────────────────────────────────────────
struct LogData {
  uint32_t t;
  float    acc_z;
  float    alt_cm;
  float    vel_z;
  float    pid_v;
  float    bat;
  int16_t  thr;
};

static const char* LOG_FILE = "/flight.bin";
static File logFile;

// ─── API ─────────────────────────────────────────────────────
void logDataSetup() {
  if (!SPIFFS.begin(true)) {
    Serial.println("[ERR] SPIFFS mount failed");
    return;
  }
  logFile = SPIFFS.open(LOG_FILE, "a");
  if (!logFile) {
    Serial.println("[ERR] Cannot open log file");
  } else {
    Serial.println("[OK]  Log file opened");
  }
}

void logData() {
  static uint32_t prevLog = 0;
  if (millis() - prevLog < 10) return;   // max 100Hz
  prevLog = millis();

  if (!logFile) return;

  LogData d;
  d.t      = millis();
  d.acc_z  = acc_z_inertial;
  d.alt_cm = current_altitude;
  d.vel_z  = VelocityVerticalKalman;
  d.pid_v  = pid_vel;
  d.bat    = bat_voltage;
  d.thr    = (int16_t)throttle;

  logFile.write((uint8_t*)&d, sizeof(d));
}

void flushLog() {
  if (logFile) logFile.flush();
}

void readLog() {
  File f = SPIFFS.open(LOG_FILE);
  if (!f) { Serial.println("No log file"); return; }

  LogData d;
  while (f.read((uint8_t*)&d, sizeof(d)) == sizeof(d)) {
    Serial.printf("t=%u acc_z=%.3f alt=%.1f vel=%.2f pid=%.2f bat=%.2f thr=%d\n",
                  d.t, d.acc_z, d.alt_cm, d.vel_z, d.pid_v, d.bat, d.thr);
  }
  f.close();
}

void clearLog() {
  if (logFile) { logFile.close(); }
  SPIFFS.remove(LOG_FILE);
  logFile = SPIFFS.open(LOG_FILE, "a");
  Serial.println("[OK]  Log cleared");
}