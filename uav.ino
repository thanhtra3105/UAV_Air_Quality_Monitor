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
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

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

float gx, gy, gz;

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

// ===== PID RATE =====
float kp_r = 0.8, ki_r = 0.000, kd_r = 0.001;
float kp_p = 0.8, ki_p = 0.000, kd_p = 0.001;
float kp_yaw = 0.8, ki_yaw = 0.000, kd_yaw = 0.001;
float kp_alt = 0.7, ki_alt = 0.002, kd_alt = 0.001;

float danh_lai = 3.0;

// ================= MOTOR =================
int throttle = 1000;
int throttle_hover = 1000;
uint16_t last_tocdo = 0;
// ================CALIBRATION MPU===============
float gyro_offset_x = 0;
float gyro_offset_y = 0;

// BMP280
float alt = 0.0;
float pressure_offset = 1013.25;

long timeout_connected, time_throttle;


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

WebServer server(80);
Adafruit_BMP280 bmp;
SimpleKalmanFilter kalmanFilter(2, 2, 0.001);

void bmp2800_setup();
void readAlt();
String pidPage() {
  String page = "<html><body>";
  page += "<h2>DRONE PID TUNING</h2>";

  // ===== JavaScript =====
  page += "<script>";
  page += "function inc(id, step, fix){";
  page += "  var e=document.getElementById(id);";
  page += "  e.value=(parseFloat(e.value)+step).toFixed(fix);";
  page += "}";
  page += "function dec(id, step, fix){";
  page += "  var e=document.getElementById(id);";
  page += "  e.value=(parseFloat(e.value)-step).toFixed(fix);";
  page += "}";
  page += "</script>";

  page += "<form action='/set'>";

  // ===== PITCH =====
  page += "<h3>PITCH</h3>";

  page += "KP_P ";
  page += "<button type='button' onclick='dec(\"kp_p\",0.01,2)'>-</button>";
  page += "<input id='kp_p' name='kp_p' value='" + String(kp_p, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_p\",0.01,2)'>+</button><br>";

  page += "KI_P ";
  page += "<button type='button' onclick='dec(\"ki_p\",0.001,3)'>-</button>";
  page += "<input id='ki_p' name='ki_p' value='" + String(ki_p, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_p\",0.001,4)'>+</button><br>";

  page += "KD_P ";
  page += "<button type='button' onclick='dec(\"kd_p\",0.001,4)'>-</button>";
  page += "<input id='kd_p' name='kd_p' value='" + String(kd_p, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_p\",0.001,4)'>+</button><br>";

  // ===== ROLL =====
  page += "<h3>ROLL</h3>";

  page += "KP_R ";
  page += "<button type='button' onclick='dec(\"kp_r\",0.01,2)'>-</button>";
  page += "<input id='kp_r' name='kp_r' value='" + String(kp_r, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_r\",0.01,2)'>+</button><br>";

  page += "KI_R ";
  page += "<button type='button' onclick='dec(\"ki_r\",0.001,4)'>-</button>";
  page += "<input id='ki_r' name='ki_r' value='" + String(ki_r, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_r\",0.001,4)'>+</button><br>";

  page += "KD_R ";
  page += "<button type='button' onclick='dec(\"kd_r\",0.001,4)'>-</button>";
  page += "<input id='kd_r' name='kd_r' value='" + String(kd_r, 4) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_r\",0.001,4)'>+</button><br>";

  // ===== YAW =====
  page += "<h3>YAW</h3>";

  page += "KP_Y ";
  page += "<button type='button' onclick='dec(\"kp_yaw\",0.01,2)'>-</button>";
  page += "<input id='kp_yaw' name='kp_yaw' value='" + String(kp_yaw, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_yaw\",0.01,2)'>+</button><br>";

  page += "KI_Y ";
  page += "<button type='button' onclick='dec(\"ki_yaw\",0.001,3)'>-</button>";
  page += "<input id='ki_yaw' name='ki_yaw' value='" + String(ki_yaw, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_yaw\",0.001,3)'>+</button><br>";

  page += "KD_Y ";
  page += "<button type='button' onclick='dec(\"kd_yaw\",0.001,3)'>-</button>";
  page += "<input id='kd_yaw' name='kd_yaw' value='" + String(kd_yaw, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_yaw\",0.001,3)'>+</button><br><br>";

  page += "<b>TUNING KP ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"kp_angle\",0.01,2)'>-</button>";
  page += "<input id='kp_angle' name='kp_angle' value='" + String(kp_angle, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kp_angle\",0.01,2)'>+</button><br><br>";

  page += "<b>TUNING KI ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"ki_angle\",0.001,3)'>-</button>";
  page += "<input id='ki_angle' name='ki_angle' value='" + String(ki_angle, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"ki_angle\",0.001,3)'>+</button><br><br>";

  page += "<b>TUNING KD ANGLE </b><br>";
  page += "<button type='button' onclick='dec(\"kd_angle\",0.001,3)'>-</button>";
  page += "<input id='kd_angle' name='kd_angle' value='" + String(kd_angle, 3) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"kd_angle\",0.001,3)'>+</button><br><br>";

  page += "<b>TARGET PITCH</b><br>";
  page += "<button type='button' onclick='dec(\"target_pitch\",1.0,1)'>-</button>";
  page += "<input id='target_pitch' name='target_pitch' value='" + String(target_pitch, 1) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"target_pitch\",1.0,1)'>+</button><br><br>";

  page += "<b>TARGET ROLL</b><br>";
  page += "<button type='button' onclick='dec(\"target_roll\",1.0,1)'>-</button>";
  page += "<input id='target_roll' name='target_roll' value='" + String(target_roll, 1) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"target_roll\",1.0,1)'>+</button><br><br>";

  // ===== DANH LAI =====
  page += "<b>DANH LAI</b><br>";
  page += "<button type='button' onclick='dec(\"danh_lai\",1,1)'>-</button>";
  page += "<input id='danh_lai' name='danh_lai' value='" + String(danh_lai, 2) + "' size='6'>";
  page += "<button type='button' onclick='inc(\"danh_lai\",1,1)'>+</button><br><br>";

  page += "<input type='submit' value='UPDATE PID'>";
  page += "</form></body></html>";

  return page;
}


void handleRoot() {
  server.send(200, "text/html", pidPage());
}

void handleSetPID() {
  if (server.hasArg("kp_p")) kp_p = server.arg("kp_p").toFloat();
  if (server.hasArg("ki_p")) ki_p = server.arg("ki_p").toFloat();
  if (server.hasArg("kd_p")) kd_p = server.arg("kd_p").toFloat();

  if (server.hasArg("kp_r")) kp_r = server.arg("kp_r").toFloat();
  if (server.hasArg("ki_r")) ki_r = server.arg("ki_r").toFloat();
  if (server.hasArg("kd_r")) kd_r = server.arg("kd_r").toFloat();

  if (server.hasArg("kp_yaw")) kp_yaw = server.arg("kp_yaw").toFloat();
  if (server.hasArg("ki_yaw")) ki_yaw = server.arg("ki_yaw").toFloat();
  if (server.hasArg("kd_yaw")) kd_yaw = server.arg("kd_yaw").toFloat();

  if (server.hasArg("kp_angle")) kp_angle = server.arg("kp_angle").toFloat();
  if (server.hasArg("ki_angle")) ki_angle = server.arg("ki_angle").toFloat();
  // if (server.hasArg("kd_yaw")) kd_yaw = server.arg("kd_yaw").toFloat();

  if (server.hasArg("target_pitch")) target_pitch = server.arg("target_pitch").toFloat();
  if (server.hasArg("target_roll")) target_roll = server.arg("target_roll").toFloat();

  if (server.hasArg("danh_lai")) {
    danh_lai = server.arg("danh_lai").toFloat();
  }


  server.send(200, "text/html",
              "<h3>PID Updated!</h3><a href='/'>Back</a>");
  Serial.println("Updated PID value");
  Serial.println("Kp_r = " + String(kp_r, 3));
  Serial.println("Ki_r = " + String(ki_r, 3));
  Serial.println("Kd_r = " + String(kd_r, 3));
  Serial.println("Kp_p = " + String(kp_p, 3));
  Serial.println("Ki_p = " + String(ki_p, 3));
  Serial.println("Kd_p = " + String(kd_p, 3));
  Serial.println("Kp_y = " + String(kp_yaw, 3));
  Serial.println("Ki_y = " + String(ki_yaw, 3));
  Serial.println("Kd_y = " + String(kd_yaw, 3));
  Serial.println("Kp_angle = " + String(kp_angle, 3));
  Serial.println("Ki_angle = " + String(ki_angle, 3));
  Serial.println("danh lai = " + String(danh_lai));
}

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
  // calib
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gyro_offset_x += g.gyro.x;
    gyro_offset_y += g.gyro.y;
    delay(2);
  }

  gyro_offset_x /= 500;
  gyro_offset_y /= 500;

  while (!compass.begin()) {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    while (1)
      ;
  }

  if (compass.isQMC()) {
    Serial.println("Initialize QMC5883");
  }

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

  lastTime = micros();

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/set", handleSetPID);
  server.begin();
}

void loop() {
  server.handleClient();

  unsigned long now = micros();
  dt = (now - lastTime) * 1e-6;
  lastTime = now;
  if (dt <= 0 || dt > 0.02) return;

  // ---- RX ----
  if (radio.available()) {
    radio.read(&rx, sizeof(rx));
    alt_hold = rx.nut1;
    if (!alt_hold) {
      throttle = map(rx.chinhtocdoquat, 0, 100, 1000, 1700);
      last_tocdo = rx.chinhtocdoquat;
    } else {
      int16_t delta = rx.chinhtocdoquat - last_tocdo;
      target_alt += delta * 0.002;  // max
      target_alt = constrain(target_alt, 0.3, 10.0);
      last_tocdo = rx.chinhtocdoquat;
    }
    target_pitch = map(rx.trucX, 0, 1023, danh_lai, -danh_lai);
    target_roll = map(rx.trucY, 0, 1023, -danh_lai, danh_lai);

    timeout_connected = millis();
  }
  if (millis() - timeout_connected > 1000)  // not connect about 2s
  {
    if (millis() - time_throttle > 300)  // 20ms
    {
      if (throttle > 1030) {
        throttle = throttle - 10;
        time_throttle = millis();
      }
    }
  }

  // Serial.println(throttle);
  
  // readAlt();
  if (alt_hold && !alt_init) {
    Serial.println("Khoi tao vi tri hien tai 1 lan");
    target_alt = current_alt;  // giữ độ cao hiện tại
    throttle_hover = throttle;
    i_alt = 0;
    alt_init = true;
  }

  if (alt_hold) {
    err_alt = target_alt - current_alt;
    i_alt += err_alt * dt;
    i_alt = constrain(i_alt, -200, 200);
    pid_alt = kp_alt * err_alt
              + ki_alt * i_alt
              + kd_alt * (err_alt - last_err_alt) / dt;

    last_err_alt = err_alt;
    pid_alt = constrain(pid_alt, -200, 200);
    throttle = throttle_hover + pid_alt;
  } else {
    // Serial.println("Che do bay thu cong");
    alt_init = false;
    pid_alt = 0;
  }
  // Serial.println(throttle);

  // ---- IMU ----
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // gx = (g.gyro.x - gyro_offset_x) * 57.2958;
  // gy = (g.gyro.y - gyro_offset_y) * 57.2958;
  gx = g.gyro.x * 57.2958;
  gy = g.gyro.y * 57.2958;
  gz = -g.gyro.z * 57.2958;
  // ===== ANGLE PITCH =====
  pitch_acc = atan2(-a.acceleration.x,
                    sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))
              * 57.2958;

  roll_acc = atan2(
               a.acceleration.y,
               sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z))
             * 57.2958;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * pitch_acc;
  roll = 0.98 * (roll + gx * dt) + 0.02 * roll_acc;

  // ================= YAW (COMPASS) =================
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();
  yaw = mag.HeadingDegress;  // DEGREE
  // yaw = kalmanFilter.updateEstimate(mag.HeadingDegress); // loc kalamn thu xem sao

  // ===== YAW HOLD INIT =====
  if (!yaw_hold_init && throttle > 1030) {
    // target_pitch = pitch;
    // target_roll = roll;
    target_yaw = yaw;
    yaw_hold_init = true;
  }
  if (throttle < 1030) {
    yaw_hold_init = false;
    target_yaw = yaw;
    i_yaw = 0;
  }


  // ===== PID ANGLE → RATE =====
  float err_angle_p = target_pitch - pitch;
  i_angle_p += err_angle_p * dt;
  i_angle_p = constrain(i_angle_p, -80, 80);
  target_rate_pitch = constrain(kp_angle * err_angle_p + ki_angle * i_angle_p, -200, 200);

  float err_angle_r = target_roll - roll;
  i_angle_r += err_angle_r * dt;
  i_angle_r = constrain(i_angle_r, -80, 80);
  target_rate_roll = constrain(err_angle_r * kp_angle + ki_angle * i_angle_r, -200, 200);


  // ===== PID RATE =====
  err_p = target_rate_pitch - gy;
  i_p += err_p * dt;
  pid_p = kp_p * err_p + ki_p * i_p + kd_p * (err_p - last_err_p) / dt;
  last_err_p = err_p;

  err_r = target_rate_roll - gx;
  i_r += err_r * dt;
  pid_r = kp_r * err_r + ki_r * i_r + kd_r * (err_r - last_err_r) / dt;
  last_err_r = err_r;

  pid_r = constrain(pid_r, -300, 300);
  pid_p = constrain(pid_p, -300, 300);

  // ================= YAW PID =================
  // angle error (wrap đúng chỗ)
  float err_angle_y = target_yaw - yaw;
  if (err_angle_y > 180) err_angle_y -= 360;
  if (err_angle_y < -180) err_angle_y += 360;

  // angle → rate
  target_rate_yaw = constrain(kp_angle * err_angle_y, -200, 200);

  // rate PID
  err_yaw = target_rate_yaw - gz;
  i_yaw += err_yaw * dt;
  pid_yaw = kp_yaw * err_yaw + ki_yaw * i_yaw + kd_yaw * (err_yaw - last_err_yaw) / dt;
  last_err_yaw = err_yaw;

  pid_yaw = constrain(pid_yaw, -300, 300);

  // ===== MIXER =====
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


  int m1 = throttle - pid_p + pid_r - pid_yaw;
  int m2 = throttle - pid_p - pid_r + pid_yaw;
  int m3 = throttle + pid_p - pid_r - pid_yaw;
  int m4 = throttle + pid_p + pid_r + pid_yaw;

  // int m1 = throttle - pid_p + pid_r;
  // int m2 = throttle - pid_p - pid_r;
  // int m3 = throttle + pid_p - pid_r;
  // int m4 = throttle + pid_p + pid_r;

  // int m1 = throttle;
  // int m2 = throttle;
  // int m3 = throttle;
  // int m4 = throttle;

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
  // Serial.println(throttle);
  // Serial.println("m1 = " + String(m1));
  // Serial.println("m2 = " + String(m2));
  // Serial.println("m3 = " + String(m3));
  // Serial.println("m4 = " + String(m4));
  // // DEBUG
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
}

void bmp2800_setup() {
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  delay(500);
  for (int i = 0; i < 5; i++) {
    pressure_offset += bmp.readPressure();
    delay(1000);
  }
  pressure_offset /= 500;
  Serial.println(pressure_offset);
}
void readAlt() {
  static uint32_t pre_time;
  if (millis() - pre_time > 500) {
    current_alt = bmp.readAltitude(bmp.readPresure/100.0, pressure_offset); /* Adjusted to local forecast! */
    pre_time = millis();
  }
}

void rxController()
{
  
}