/*
So do drone nhu nay:

m4 (CCW)     m1 (CW)
    \       /
      ^
      |
    (nose)
      |
    /       \
m3 (CW)      m2 (CCW)

*/


#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <DFRobot_QMC5883.h>
#include "pidWebPage.h"
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>
#include "kalman_filter.h"
#include "poshold.h"
#include "log.h"
#include "gy_tof.h"

#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.017453f

#define MPU_ADDR 0x68
#define PWM_FREQ 500
#define PWM_RES 12

#define MOSI 4
#define SCK 5
#define MISO 6
#define CE_PIN 7
#define CSN_PIN 8

#define SDA_PIN 2
#define SCL_PIN 3

#define M1_PIN 0
#define M2_PIN 1
#define M3_PIN 9
#define M4_PIN 10

#define GRAVITY 9.81

float gx, gy, gz;
float ax, ay, az;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;
float gx_offset = 0.0;
float gy_offset = 0.0;
float gz_offset = 0.0;

// ===== ANGLE =====
float i_angle_p = 0, i_angle_r = 0;
float pitch = 0, roll = 0, yaw = 0;
float pitch_acc = 0, roll_acc = 0;

float target_pitch = 0, target_roll = 0;  // độ (angle mode)
float target_rate_pitch = 0;              // deg/s
float target_rate_roll = 0;
float target_rate_yaw = 0;

float target_yaw = 0;
float target_alt = 0;
float current_alt = 0;

// ===== TIME =====
unsigned long lastTime;
float dt;

float Iterm, Pterm, Dterm;  // for pid_equation
float kp_vel_z = 3.5, ki_vel_z = 0.0015, kd_vel_z = 0.01, pre_vel_z_err = 0;

// ===== PID ANGLE =====
float kp_angle = 6.5, ki_angle = 0, kd_angle = 0;

// ===== PID VAR =====
float err_r, err_p, err_yaw, err_alt;
float last_err_r = 0, last_err_p = 0, last_err_yaw = 0, last_err_alt;
float i_r = 0, i_p = 0, i_yaw = 0, i_alt;
float pid_r, pid_p, pid_yaw, pid_alt;


bool yaw_hold_init = false;
bool alt_hold;  // nut 1 = giữ độ cao, nut 2 = bay tay
bool alt_init = false;
bool pos_hold_active = false;

// ===== PID RATE =====
float kp_r = 0.85, ki_r = 0.000, kd_r = 0.0015;
float kp_p = 0.85, ki_p = 0.000, kd_p = 0.0015;
float kp_yaw = 0.85, ki_yaw = 0.000, kd_yaw = 0.0015;
float kp_alt = 0.7, ki_alt = 0.002, kd_alt = 0.001;

float kp_vz = 80, ki_vz = 20, kd_vz = 0;

float i_vz, pid_vz;
float last_err_vz = 0.0;
float target_vz = 0;
float last_alt = 0;

float danh_lai = 4.0;


// --- CÁC BIẾN TOÀN CỤC ---
float estimated_altitude = 0.0f;       // cm
float estimated_velocity = 0.0f;       // cm/s
float last_estimated_altitude = 0.0f;  // cm

#define ALPHA 0.95f
// #define DT 0.01f // Thời gian vòng lặp (giả sử 100Hz -> 0.01s)

// ================= MOTOR =================
int throttle = 1000;
int throttle_hover = 1000;
uint16_t last_tocdo = 0;
// ================CALIBRATION MPU===============
float gyro_offset_x = 0;
float gyro_offset_y = 0;

// BMP280
float alt = 0.0;
float alt_offset = 1013.25;

long timeout_connected, time_throttle;

// POS HOLD
// float hold_lat, hold_lon;
uint8_t log_flag = 0;
float pid_vel, acc_z_inertial;
float preIterm = 0;

// ================= NRF =================
struct ControlData {
  uint16_t trucX;
  uint16_t trucY;
  uint8_t trai;
  uint8_t phai;
  uint8_t len;
  uint8_t xuong;
  uint8_t batquathut;
  uint8_t nut1;
  uint8_t nut2;
  uint8_t chinhtocdoquat;
};

ControlData rx;
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
Adafruit_MPU6050 mpu;
DFRobot_QMC5883 compass(&Wire, /*I2C addr*/ QMC5883_ADDRESS);

const char* ssid = "DRONE";
const char* password = "L02012001";


Adafruit_BMP280 bmp;
SimpleKalmanFilter kalmanFilter(2, 2, 0.001);

void bmp2800_setup();
void readAlt();


// ================= PWM =================
uint32_t usToDuty(int us) {
  us = constrain(us, 1000, 2000);
  return map(us, 1000, 2000, 2048, 4095);
}

void writeMotor(int ch, int us) {
  us = constrain(us, 1000, 2000);
  ledcWrite(ch, usToDuty(us));
}

void yawRead() {
  float declinationAngle = (-1.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degress = ");
  Serial.println(mag.HeadingDegress);
}
void setup() {
  Serial.begin(115200);
  gpsSetup();
  SPI.begin(SCK, MISO, MOSI, CSN_PIN);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU FAIL");
    while (1)
      ;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // for (int i = 0; i < 500; i++) {
  //   sensors_event_t a, g, t;
  //   mpu.getEvent(&a, &g, &t);
  //   ax_offset += a.acceleration.x;
  //   ay_offset += a.acceleration.y;
  //   az_offset += a.acceleration.z;
  //   delay(5);
  // }
  // ax_offset = (ax_offset / 500);
  // ay_offset = (ay_offset / 500);
  // az_offset = (az_offset / 500);
  // 
  ax_offset = 0.09940;
  ay_offset = 0.04193;
  az_offset = 9.33029;
  gx_offset = -0.08780;
  gy_offset = 0.02199;
  gz_offset = 0.00245;
  Serial.print("Calib az done with ax, ay, az offset: ");
  Serial.print(ax_offset);
  Serial.print(",");
  Serial.print(ay_offset);
  Serial.print(",");
  Serial.println(az_offset);
  delay(2000);
  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    while (1)
      ;
  }

  if (compass.isQMC()) {
    Serial.println("Initialize QMC5883");
  }

  bmp280_setup();

  ledcAttachPin(M1_PIN, 0);
  ledcAttachPin(M2_PIN, 1);
  ledcAttachPin(M3_PIN, 2);
  ledcAttachPin(M4_PIN, 3);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcSetup(3, PWM_FREQ, PWM_RES);

  // Arm ESC
  for (int i = 0; i < 4; i++) {
    writeMotor(0, 1000);
    writeMotor(1, 1000);
    writeMotor(2, 1000);
    writeMotor(3, 1000);
    delay(10);
  }
  delay(2000);


  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  WiFi.softAP(ssid, password);
  web_setup();
  Serial.println("WiFi AP started");
  Serial.println(WiFi.softAPIP());

  kalman_setup();

  delay(100);
  static float pre_K_alt = 0;
  static float pre_K_vel = 0;

  // logDataSetup();
  while (1) {
    if (radio.available()) {
      radio.read(&rx, sizeof(rx));
      if (rx.chinhtocdoquat == 0)
        break;
    }
  }

  Serial.println("START");
  delay(100);
  lastTime = micros();
}

void loop() {
  // server.handleClient();

  unsigned long now = micros();
  dt = (now - lastTime) * 1e-6;  // filter
  // Serial.println(dt, 5);
  lastTime = now;
  if (dt <= 0 || dt > 0.04) return;

  // ---- RX ----
  rxController();
  // Serial.println(throttle);
  // ===== YAW HOLD INIT =====
  if (!yaw_hold_init && throttle > 1030) {
    target_yaw = yaw;
    // home_position = gpsRead();
    yaw_hold_init = true;
  }
  if (throttle < 1030) {
    yaw_hold_init = false;
    target_yaw = yaw;
    i_yaw = 0;
  }
  calculateAngle();

  // acc_z_inertial = -sin(pitch * (3.142f / 180.0f)) * ax
  //                  + cos(pitch * (3.142f / 180.0f)) * sin(roll * (3.142f / 180.0f)) * ay
  //                  + cos(pitch * (3.142f / 180.0f)) * cos(roll * (3.142f / 180.0f)) * az;
  // acc_z_inertial = (acc_z_inertial - az_offset) * 100.0f;  // cm/s2
  // float bmp_cm = (bmp.readAltitude() - alt_ofgFlightet) * 100.0f;
  
  // kalman_2d(acc_z_inertial, current_alt);

  calculateAnglePID();
  calculateRatePID();

  float tof_cm = readTOF()/10;
  float roll_rad  = roll * DEG_TO_RAD;
  float pitch_rad = pitch * DEG_TO_RAD;
  current_alt = tof_cm * cosf(roll_rad) * cosf(pitch_rad);
  // ===== HOLD VELOCITY =======
  const int BASE_HOVER_THROTTLE = 1450;

  if (alt_hold && !alt_init) {
    Serial.println("Khoi tao vi tri hien tai 1 lan");
    alt_init = true;
    preIterm = 0;
    target_alt = current_alt;
    // hold_lat = current_lat;
    // hold_lon = current_lon;
  }
  if (alt_hold) {
    // float target_vel = 0;
    // if (rx.chinhtocdoquat > 70) {
    //   target_vel = map(rx.chinhtocdoquat, 55, 100, 0, 80);  // Tốc độ lên tối đa 100cm/s
    // } else if (rx.chinhtocdoquat < 30) {
    //   target_vel = map(rx.chinhtocdoquat, 0, 45, -80, 0);  // Tốc độ xuống tối đa -100cm/s
    // }
    // // Serial.print("target_vel: ");
    // // Serial.println(target_vel);
    // pid_vel = calVelPID(target_vel);
    // throttle = BASE_HOVER_THROTTLE + pid_vel;
    // if (throttle > 1800) throttle = 1800;
    // logData();

    float alt_err = target_alt - current_alt;
    pid_alt = pid_equation(err_alt, 6, 0, 1, last_err_alt, 0);
    last_err_alt = alt_err;
    // Serial.println(pid_alt);7


  } else {
    alt_init = false;
  }

  mixer();  // mix and control motor

  // debug();
  // Serial.println(pitch);
  // Serial.printf("pitch=%.2f roll=%.2f\n",pitch, roll);
}

void bmp280_setup() {
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  delay(500);
  double sum = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    sum += bmp.readAltitude();
    delay(10);
  }
  alt_offset = (float)(sum / N);
  Serial.printf("[OK]  BMP280 — alt_offset = %.2f m\n", alt_offset);
}
void readAlt() {
  static uint32_t pre_time;
  if (millis() - pre_time > 50) {
    current_alt = (bmp.readAltitude() - alt_offset) * 100;  //cm
  }
}

void rxController() {
  if (radio.available()) {
    radio.read(&rx, sizeof(rx));
    alt_hold = rx.nut1;
    log_flag = rx.nut2;
    if (!alt_hold) {
      throttle = map(rx.chinhtocdoquat, 0, 100, 1000, 1700);
      pos_hold_active = false;
    } else {

      // pos_hold_active = true;
    }
    // if (!pos_hold_active) {
    //   target_pitch = map(rx.trucX, 0, 1023, danh_lai, -danh_lai);
    //   target_roll = map(rx.trucY, 0, 1023, -danh_lai, danh_lai);
    // }
    target_pitch = map(rx.trucX, 0, 1023, danh_lai, -danh_lai);
    target_roll = map(rx.trucY, 0, 1023, -danh_lai, danh_lai);
    // Serial.print("truc X: ");
    // Serial.println(rx.trucX);
    timeout_connected = millis();
  }

  if (millis() - timeout_connected > 1000)  // not connect about 2s
  {
    if (millis() - time_throttle > 400)  // 20ms
    {
      alt_hold = false;
      if (throttle > 1250) {

        throttle = throttle - 10;
        time_throttle = millis();
      }
    }
  }
}

void calculateAngle() {
  // ---- IMU ----
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);


  gx = (g.gyro.x - gx_offset) * 57.2958;
  gy = (g.gyro.y - gy_offset) * 57.2958;
  gz = -(g.gyro.z - gz_offset) * 57.2958;

  ax = a.acceleration.x - ax_offset;
  ay = a.acceleration.y - ay_offset;
  az = a.acceleration.z - (az_offset - 9.81);
  // // ===== ANGLE PITCH =====
  pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;
  roll_acc = atan2(ay, sqrt(ax * ax + az * az)) * 57.2958;
  /* COMPLEMENTARY FILTER*/
  // pitch = 0.98 * (pitch + gy * dt) + 0.02 * pitch_acc;
  // roll = 0.98 * (roll + gx * dt) + 0.02 * roll_acc;

  /*======== KALMAN FILTER =======*/
  // Cho Roll
  kalman_1d(roll, KalmanUncertaintyAngleRoll, gx, roll_acc);
  roll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // Cho Pitch
  kalman_1d(pitch, KalmanUncertaintyAnglePitch, gy, pitch_acc);
  pitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Serial.printf("pitch=%.2f, roll=%.2f\n", pitch, roll);
  // ================= YAW (COMPASS) =================
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  yaw = mag.HeadingDegress;  // DEGREE

  // pitch = pitch - 0.3;
  // roll = roll - 0.7;
}

void calculateAnglePID() {
  // ===== PITCH PID ANGLE =====
  float err_angle_p = target_pitch - pitch;
  i_angle_p += err_angle_p * dt;
  i_angle_p = constrain(i_angle_p, -80, 80);
  target_rate_pitch = constrain(kp_angle * err_angle_p + ki_angle * i_angle_p, -200, 200);

  // ===== ROLL PID ANGLE =====
  float err_angle_r = target_roll - roll;
  i_angle_r += err_angle_r * dt;
  i_angle_r = constrain(i_angle_r, -80, 80);
  target_rate_roll = constrain(err_angle_r * kp_angle + ki_angle * i_angle_r, -200, 200);

  // ===== YAW PID ANGLE =====
  float err_angle_y = target_yaw - yaw;
  if (err_angle_y > 180) err_angle_y -= 360;
  if (err_angle_y < -180) err_angle_y += 360;
  target_rate_yaw = constrain(kp_angle * err_angle_y, -200, 200);
}

void calculateRatePID() {
  // ================= PITCH RATE PID =================
  err_p = target_rate_pitch - gy;
  i_p += err_p * dt;
  pid_p = kp_p * err_p + ki_p * i_p + kd_p * (err_p - last_err_p) / dt;
  last_err_p = err_p;

  // ================= ROLL RATE PID =================
  err_r = target_rate_roll - gx;
  i_r += err_r * dt;
  pid_r = kp_r * err_r + ki_r * i_r + kd_r * (err_r - last_err_r) / dt;
  last_err_r = err_r;

  pid_r = constrain(pid_r, -300, 300);
  pid_p = constrain(pid_p, -300, 300);

  // ================= YAW RATE PID =================
  err_yaw = target_rate_yaw - gz;
  i_yaw += err_yaw * dt;
  pid_yaw = kp_yaw * err_yaw + ki_yaw * i_yaw + kd_yaw * (err_yaw - last_err_yaw) / dt;
  last_err_yaw = err_yaw;

  pid_yaw = constrain(pid_yaw, -300, 300);
}

void mixer() {
  int m1 = throttle - pid_p + pid_r - pid_yaw - pid_alt;
  int m2 = throttle - pid_p - pid_r + pid_yaw - pid_alt;
  int m3 = throttle + pid_p - pid_r - pid_yaw - pid_alt;
  int m4 = throttle + pid_p + pid_r + pid_yaw - pid_alt;
  // int m1 = throttle - pid_p ;
  // int m2 = throttle - pid_p;
  // int m3 = throttle + pid_p;
  // int m4 = throttle + pid_p;
  if (throttle > 1030) {
    writeMotor(0, m1);
    writeMotor(1, m2);
    writeMotor(2, m3);
    writeMotor(3, m4);
  } else {
    writeMotor(0, 1000);
    writeMotor(1, 1000);
    writeMotor(2, 1000);
    writeMotor(3, 1000);
  }
}
void debug() {
  // // DEBUG

  // int m1 = throttle - pid_p ;
  // int m2 = throttle - pid_p;
  // int m3 = throttle + pid_p;
  // int m4 = throttle + pid_p;

  // int m1 = throttle + pid_r;
  // int m2 = throttle - pid_r;
  // int m3 = throttle - pid_r;
  // int m4 = throttle + pid_r;

  // int m1 = throttle - pid_yaw;
  // int m2 = throttle + pid_yaw;
  // int m3 = throttle - pid_yaw;
  // int m4 = throttle + pid_yaw;

  // int m1 = throttle - pid_p + pid_r;
  // int m2 = throttle - pid_p - pid_r;
  // int m3 = throttle + pid_p - pid_r;
  // int m4 = throttle + pid_p + pid_r;

  // int m1 = throttle;
  // int m2 = throttle;
  // int m3 = throttle;
  // int m4 = throttle;


  // Serial.println(throttle);
  // Serial.println("m1 = " + String(m1));
  // Serial.println("m2 = " + String(m2));
  // Serial.println("m3 = " + String(m3));
  // Serial.println("m4 = " + String(m4));

  // Serial.print("Pitch:");
  // Serial.print(pitch);
  // Serial.print("  TargetRate:");
  // Serial.print(target_rate_pitch);
  // Serial.print("  PID_P:");
  // Serial.println(pid_p);

  // Serial.print("  roll:");
  // Serial.print(roll);
  // Serial.print("  TargetRate:");
  // Serial.print(target_rate_roll);
  // Serial.print("  PID_R:");
  // Serial.println(pid_r);

  // delay(100);
  // Serial.print("yaw:");
  // Serial.print(yaw);
  // Serial.print("  TargetRate:");
  // Serial.print(target_yaw);
  // Serial.print("  PID_Y:");
  // Serial.println(pid_yaw);
  // delay(50);

  // Serial.print("  pitch: ");
  // Serial.print(pitch);
  // Serial.print("  roll: ");
  // Serial.print(roll);
  // Serial.print("  ax: ");
  // Serial.println(ax);
  // Serial.print("  ay: ");
  // Serial.println(ay);
  // Serial.print("  az: ");
  // Serial.println(az);


  /*DEBUG Altitude Hold MODE*/
  // Serial.print("  K_vel: ");
  // Serial.print(K(1, 0), 12);
  // Serial.println(K);
  // Serial.print(" current alt: ");
  // Serial.print(current_alt);
  // Serial.print(" alt_kalman: ");
  // Serial.print(AltitudeKalman);
  // Serial.print(" vel_kalman: ");
  // Serial.print(VelocityVerticalKalman);
  // Serial.print(" PID_vel: ");
  // Serial.print(pid_vel);
  // Serial.print("  throttle: ");
  // Serial.println(throttle);
  // Serial.println(log_flag);


  // if (log_flag) {
  //   Serial.println("read log");
  //   readLog();
  //   // SPIFFS.remove(fileName);  // xoá log cũ
  //   while (1) {
  //     rxController();

  //     if (!log_flag) {
  //       clearLog();
  //       break;
  //     }
  //   }
  // }
}


float pid_equation(float err, float kP, float kI, float kD, float preE, float preI) {
  Pterm = kP * err;
  Iterm = preI + kI * (err + preE) * dt / 2;
  Iterm = constrain(Iterm, -400, 400);

  Dterm = kD * (err - preE);
  return (Pterm + Iterm + Dterm);
}

float calVelPID(float vel_z_target) {
  float vel_z_err = vel_z_target - VelocityVerticalKalman;

  float vel_z_PID = pid_equation(vel_z_err, kp_vel_z, ki_vel_z, kd_vel_z, pre_vel_z_err, preIterm);
  vel_z_PID = constrain(vel_z_PID, -300, 300);
  pre_vel_z_err = vel_z_err;
  preIterm = Iterm;
  return vel_z_PID;
}


// float processAltitudeMeasurement(int tof_mm,
//                                  float roll_deg,
//                                  float pitch_deg,
//                                  float bmp_cm,
//                                  float vz_cm_s,
//                                  float dt,
//                                  bool &tof_ok_out,
//                                  float &R_out_cm2) {

//   tof_ok_out = false;
//   R_out_cm2 = R_BMP;

//   if (tof_mm <= 0 || tof_mm > 9500) {
//     return bmp_cm;
//   }

//   float tof_raw_cm = tof_mm * 0.1f;

//   // Nếu nghiêng lớn, tia TOF nhìn lệch nhiều, không nên tin.
//   if (fabsf(roll_deg) > 25.0f || fabsf(pitch_deg) > 25.0f) {
//     return bmp_cm;
//   }

//   // Bù nghiêng: TOF đo đường xiên, cần chiếu về phương thẳng đứng.
//   float roll_rad  = roll_deg * DEG_TO_RAD;
//   float pitch_rad = pitch_deg * DEG_TO_RAD;
//   float tof_cm = tof_raw_cm * cosf(roll_rad) * cosf(pitch_rad);

//   if (!last_tof_ok) {
//     last_tof_cm = tof_cm;
//     last_tof_ok = true;
//     tof_ok_out = true;
//     R_out_cm2 = R_TOF_GOOD;
//     return 0.85f * tof_cm + 0.15f * bmp_cm;
//   }

//   float delta = tof_cm - last_tof_cm;
//   float rate = delta / constrain(dt, 0.02f, 0.2f);

//   // TOF nhảy quá nhanh so với động học Z của drone: reject.
//   if (fabsf(rate) > 1000.0f) {  // lech 1m 
//     R_out_cm2 = R_BMP;
//     return bmp_cm;
//   }

//   // TOF giảm mạnh nhưng velocity Z không cho thấy drone đang rơi nhanh:
//   // khả năng cao là cây/cỏ/vật cản bên dưới.
//   if (delta < -50.0f && vz_cm_s > -120.0f) {
//     R_out_cm2 = R_BMP;
//     return bmp_cm;
//   }

//   // TOF tăng mạnh: có thể gặp vùng trũng. Không bỏ hoàn toàn, nhưng giảm độ tin cậy.
//   bool terrain_step_suspected = (delta > 80.0f && fabsf(vz_cm_s) < 120.0f);

//   // Low-pass nhẹ cho TOF.
//   float tof_filtered = 0.75f * last_tof_cm + 0.25f * tof_cm;    // cm
//   last_tof_cm = tof_filtered;
//   tof_ok_out = true;

//   if (terrain_step_suspected) {
//     R_out_cm2 = R_TOF_BAD;
//     return 0.30f * tof_filtered + 0.70f * bmp_cm;
//   }

//   float w_tof = 0.0f;

//   if (tof_filtered < 300.0f) {
//     w_tof = 0.98f;
//   } else if (tof_filtered < 800.0f) {
//     w_tof = 0.95f;
//   } else if (tof_filtered < 1000.0f) {
//     float alpha = (tof_filtered - 800.0f) / 200.0f;
//     alpha = constrain(alpha, 0.0f, 1.0f);
//     w_tof = 0.95f + alpha * (0.50f - 0.95f);
//   } else {
//     w_tof = 0.0f;
//   }

//   float w_bmp = 1.0f - w_tof;

//   R_out_cm2 =
//     w_tof * w_tof * R_TOF_GOOD +
//     w_bmp * w_bmp * R_BMP;

//   return w_tof * tof_filtered + w_bmp * bmp_cm;
// }