#include "SPIFFS.h"

const char* fileName = "/flight.bin";

File logFile;

extern float az,
  acc_z_inertial,
  current_alt,
  VelocityVerticalKalman,
  pid_vel;
extern int throttle;

struct LogData {
  uint32_t t;
  float az;
  float acc_z;
  float alt;
  float vel;
  float pid;
  int throttle;
};

void logDataSetup() {
  SPIFFS.begin(true);
  // if (SPIFFS.exists(fileName)) {
  //   SPIFFS.remove(fileName);  // xoá log cũ
  // }
  logFile = SPIFFS.open(fileName, "a");
}
void logData() {
  static uint32_t preLog;

  if (millis() - preLog < 10) return;   // 100Hz
  preLog = millis();

  LogData d;

  d.t = millis();
  d.az = az;
  d.acc_z = acc_z_inertial;
  d.alt = current_alt;
  d.vel = VelocityVerticalKalman;
  d.pid = pid_vel;
  d.throttle = throttle;

  logFile.write((uint8_t*)&d, sizeof(d));
}

void readLog() {

  File file = SPIFFS.open("/flight.bin");

  LogData d;

  while (file.read((uint8_t*)&d, sizeof(d)) == sizeof(d)) {

    Serial.print("time: ");
    Serial.print(d.t);

    Serial.print(" az: ");
    Serial.print(d.az);

    Serial.print(" az_iner: ");
    Serial.print(d.acc_z);
    
    Serial.print(" alt: ");
    Serial.print(d.alt);

    Serial.print(" vel: ");
    Serial.print(d.vel);

    Serial.print(" pid_vel: ");
    Serial.print(d.pid);

    Serial.print(" throttle: ");
    Serial.println(d.throttle);
  }

  file.close();
}

void clearLog() {
  if (SPIFFS.exists(fileName)) {
    SPIFFS.remove(fileName);  // xoá log cũ
  }
}