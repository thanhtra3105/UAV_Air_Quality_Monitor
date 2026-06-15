/*
  ╔══════════════════════════════════════════════════════════╗
  ║              QUADCOPTER FLIGHT CONTROLLER                ║
  ║                  ESP32 + FreeRTOS                        ║
  ╠══════════════════════════════════════════════════════════╣
  ║  Motor layout (top view):                               ║
  ║    M4 (CCW) ──── FRONT ──── M1 (CW)                    ║
  ║       \                    /                            ║
  ║        \        ↑         /                             ║
  ║         \     (nose)     /                              ║
  ║        /                  \                             ║
  ║    M3 (CW)             M2 (CCW)                         ║
  ╠══════════════════════════════════════════════════════════╣
  ║  Task structure:                                         ║
  ║   Core 1 | TaskRateControl    500Hz  Priority 6          ║
  ║   Core 1 | TaskAngleControl   100Hz  Priority 5          ║
  ║   Core 0 | TaskAltitudeSensor 50Hz   Priority 4          ║
  ║   Core 1 | TaskCommunication  20Hz   Priority 3          ║
  ║   Core 0 | TaskTelemetry      10Hz   Priority 2          ║
  ║   Core 0 | TaskWebServer        -    Priority 1          ║
  ╚══════════════════════════════════════════════════════════╝
*/


#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <DFRobot_QMC5883.h>
#include <Adafruit_BMP280.h>
#include "types.h"
#include "pidWebPage.h"
#include "kalman_filter.h"
#include "poshold.h"
#include "log.h"
#include "ina219.h"
#include "gy_tof.h"

// ════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ════════════════════════════════════════════════════════════
#define MOSI_PIN 4
#define SCK_PIN 5
#define MISO_PIN 6
#define CE_PIN 7
#define CSN_PIN 8

#define SDA_PIN 2
#define SCL_PIN 3

#define M1_PIN 0   // CW  - Front Right
#define M2_PIN 1   // CCW - Back  Right
#define M3_PIN 9   // CW  - Back  Left
#define M4_PIN 10  // CCW - Front Left

// ════════════════════════════════════════════════════════════
//  CONSTANTS
// ════════════════════════════════════════════════════════════
#define PWM_FREQ 500
#define PWM_RES 12
#define PWM_MIN 2048  // duty @ 1000µs
#define PWM_MAX 4095  // duty @ 2000µs
#define GRAVITY 9.81f
#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.017453f

// ════════════════════════════════════════════════════════════
//  HARDWARE OBJECTS
// ════════════════════════════════════════════════════════════
RF24 radio(CE_PIN, CSN_PIN);
Adafruit_MPU6050 mpu;
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);
Adafruit_BMP280 bmp;

const byte RF24_ADDRESS[6] = "00001";
const char *WIFI_SSID = "DRONE";
const char *WIFI_PASS = "L02012001";

float declinationAngle = (-1.0f + (26.0f / 60.0f)) / (180.0f / PI);

// ════════════════════════════════════════════════════════════
//  IMU CALIBRATION (đo trước, ghi cứng vào đây)
// ════════════════════════════════════════════════════════════
// gx_offset = -0.08777;
//   gy_offset = 0.01915;
//   gz_offset = 0.00697;

//   ax_offset = 0.12731;
//   ay_offset = 0.00210;
//   az_offset = 9.39676;
/*
Calib az done with ax, ay, az offset: 0.05692,0.02593,9.35071
Calib gyro done with gx, gy, gz offset: -0.08704,0.02099,0.00579
*/
const float GX_OFgFlightET = -0.08704f;
const float GY_OFgFlightET = 0.02099f;
const float GZ_OFgFlightET = 0.00579f;
const float AX_OFgFlightET = 0.05692f;
const float AY_OFgFlightET = 0.02593f;
const float AZ_OFgFlightET = 9.35071f;  // bao gồm gravity khi nằm bằng

// ════════════════════════════════════════════════════════════
//  REMOTE CONTROL PACKET
// ════════════════════════════════════════════════════════════
struct ControlData {
  uint16_t trucX;  // Joystick trái (pitch)
  uint16_t trucY;  // Joystick phải (roll)
  uint8_t trai;
  uint8_t phai;
  uint8_t len;
  uint8_t xuong;
  uint8_t batquathut;
  uint8_t nut1;            // Alt hold ON/OFF
  uint8_t nut2;            // Pos hold ON/OFF
  uint8_t chinhtocdoquat;  // Throttle 0–100
};

// ════════════════════════════════════════════════════════════
//  SHARED STATE — bảo vệ bằng mutex tương ứng
//  Quy ước:
//    xImuMutex   → imu_data   (ax/ay/az/gx/gy/gz)
//    xFlightMutex→ flight_state (angles, rates, PIDs, throttle, modes)
//    xGpsMutex   → gps state  (trong poshold.cpp)
// ════════════════════════════════════════════════════════════

// --- IMU raw (ghi bởi TaskRateControl, đọc bởi TaskAngleControl) ---
struct ImuData {
  float gx, gy, gz;  // deg/s
  float ax, ay, az;  // m/s²
};
static ImuData imu_data;


FlightState gFlight;

PidGains gains;
// Altitude measurement/fusion
float alt_ofgFlightet = 0.0f;
// Altitude measurement/fusion
const float R_TOF_GOOD = 25.0f;     // std ≈ 5cm
const float R_TOF_BAD  = 900.0f;    // std ≈ 30cm
const float R_BMP      = 2500.0f;   // std ≈ 50cm

volatile float current_altitude = 0.0f;       // cm
volatile float altitude_meas_R_cm2 = R_BMP;   // cm^2
volatile bool altitude_meas_new = false;

float bmp_altitude_cm = 0.0f;
float tof_altitude_cm = 0.0f;
volatile bool tof_valid = false;

// Vòng ngoài altitude: sai số độ cao cm -> target velocity cm/s
const float ALT_POS_KP = 0.08f;
const float ALT_TARGET_VEL_LIMIT = 100.0f;  // cm/s
static float last_vz_for_d = 0.0f;

// TOF filtering state
static float last_tof_cm = 0.0f;
static bool last_tof_ok = false;

// Battery
float bat_voltage = 12.0f;
float bat_full = 12.2f;

// PID integrator state (chỉ dùng trong flight tasks, bảo vệ bởi xFlightMutex)
static float i_r = 0, i_p = 0, i_yaw = 0;
static float i_angle_p = 0, i_angle_r = 0;
static float last_err_r = 0, last_err_p = 0, last_err_yaw = 0;
// Altitude PID state
static float pre_vel_z_err = 0, pre_vel_z_iterm = 0;

// Timeout kết nối
static unsigned long timeout_connected = 0;
static unsigned long time_throttle_drop = 0;

// Waypoint / pos hold
static uint8_t nut2_previous = 0;
extern float gps_roll_pid_adjust, gps_pitch_pid_adjust;
extern uint8_t waypoint_set;

// Log
volatile uint8_t log_flag = 0;

// ─── Bridge variables cho log.h và poshold.cpp ────────────────
// log.h khai báo extern các biến này — trỏ vào gFlight
float acc_z_inertial = 0.0f;   // sync từ gFlight.acc_z_inertial trong TaskTelemetry
float pid_vel = 0.0f;          // sync từ gFlight.pid_vel
volatile int throttle = 1000;  // sync từ gFlight.throttle
float yaw = 0.0f;              // sync từ gFlight.yaw (dùng bởi poshold.cpp)

// Mode
// ─── WebServer (extern trong pidWebPage.h) ────────────────────
#include <WiFi.h>
#include <WebServer.h>
WebServer server(80);

// BASE_HOVER_THROTTLE dùng cho altitude hold
uint16_t BASE_HOVER_THROTTLE = 1450;

// ════════════════════════════════════════════════════════════
//  RTOS HANDLES
// ════════════════════════════════════════════════════════════
SemaphoreHandle_t xImuMutex;     // bảo vệ imu_data
SemaphoreHandle_t xFlightMutex;  // bảo vệ gFlight (FlightState)
SemaphoreHandle_t xGainsMutex;   // bảo vệ gains (PidGains) — WebServer ghi
SemaphoreHandle_t xI2CMutex;     // bảo vệ I2C bus
SemaphoreHandle_t xAltMutex;

QueueHandle_t xControlQueue;  // Comm → flight tasks (ControlData)

TaskHandle_t TaskRateHandle;
TaskHandle_t TaskAngleHandle;

// ════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ════════════════════════════════════════════════════════════
void TaskRateControl(void *pvParameters);
void TaskAngleControl(void *pvParameters);
void TaskAltitudeSensor(void *pvParameters);
void TaskCommunication(void *pvParameters);
void TaskTelemetry(void *pvParameters);
void TaskWebServer(void *pvParameters);

void bmp280_setup();
void readAlt();
void readYaw_safe();
float processAltitudeMeasurement(int tof_mm,
                                 float roll_deg,
                                 float pitch_deg,
                                 float bmp_cm,
                                 float vz_cm_s,
                                 float dt,
                                 bool &tof_ok_out,
                                 float &R_out_cm2);
// ════════════════════════════════════════════════════════════
//  PWM HELPERS
// ════════════════════════════════════════════════════════════
static inline uint32_t usToDuty(int us) {
  us = constrain(us, 1000, 2000);
  return (uint32_t)map(us, 1000, 2000, PWM_MIN, PWM_MAX);
}

static inline void writeMotor(int ch, int us) {
  ledcWrite(ch, usToDuty(us));
}

static inline void motorsOff() {
  for (int ch = 0; ch < 4; ch++) ledcWrite(ch, usToDuty(1000));
}

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // GPS trên UART1
  gpsSetup();

  // SPI (NRF24)
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);

  // I2C (MPU6050, QMC5883, BMP280, INA219)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // ── MPU6050 ──────────────────────────────────────────────
  if (!mpu.begin()) {
    Serial.println("[ERR] MPU6050 not found!");
    while (1) vTaskDelay(1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.printf("[OK]  MPU6050 — ofgFlightets ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f\n",
                AX_OFgFlightET, AY_OFgFlightET, AZ_OFgFlightET, GX_OFgFlightET, GY_OFgFlightET, GZ_OFgFlightET);

  // ── QMC5883 ──────────────────────────────────────────────
  int compassRetry = 0;
  while (!compass.begin()) {
    if (++compassRetry > 10) {
      Serial.println("[ERR] QMC5883 not found!");
      while (1) vTaskDelay(1);
    }
    delay(200);
  }
  if (compass.isQMC()) Serial.println("[OK]  QMC5883");
  compass.setDeclinationAngle(declinationAngle);

  // ── BMP280 ───────────────────────────────────────────────
  bmp280_setup();

  // ── ESC init ─────────────────────────────────────────────
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_PIN, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_PIN, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcAttachPin(M3_PIN, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES);
  ledcAttachPin(M4_PIN, 3);
  motorsOff();
  delay(500);
  Serial.println("[OK]  ESC armed");

  // ── NRF24 ────────────────────────────────────────────────
  radio.begin();
  radio.openReadingPipe(0, RF24_ADDRESS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  Serial.println("[OK]  NRF24L01");

  // ── Kalman setup ─────────────────────────────────────────
  kalman_setup();

  // ── Log SPIFgFlight ───────────────────────────────────────────
  logDataSetup();

  // ── Chờ throttle = 0 trước khi bay ──────────────────────
  Serial.println("[..] Waiting for throttle = 0 ...");
  {
    ControlData tmp;
    while (true) {
      if (radio.available()) {
        radio.read(&tmp, sizeof(tmp));
        if (tmp.chinhtocdoquat == 0) break;
      }
      delay(10);
    }
  }
  Serial.println("[OK]  Throttle zero confirmed");

  // ════════════════════════════════════════════════════════
  //  Tạo mutex & queue TRƯỚC khi tạo task
  // ════════════════════════════════════════════════════════
  xImuMutex = xSemaphoreCreateMutex();
  xFlightMutex = xSemaphoreCreateMutex();
  xGainsMutex = xSemaphoreCreateMutex();
  xI2CMutex = xSemaphoreCreateMutex();
  xAltMutex = xSemaphoreCreateMutex();
  xControlQueue = xQueueCreate(2, sizeof(ControlData));

  configASSERT(xImuMutex);
  configASSERT(xFlightMutex);
  configASSERT(xGainsMutex);
  configASSERT(xI2CMutex);
  configASSERT(xAltMutex);
  configASSERT(xControlQueue);

  // ════════════════════════════════════════════════════════
  //  Tạo task
  // ════════════════════════════════════════════════════════
  //                              name          stack   param pri   handle    core
  xTaskCreate(TaskRateControl,     "Rate",  8192, NULL, 6, &TaskRateHandle);
  xTaskCreate(TaskAngleControl,    "Angle", 8192, NULL, 5, &TaskAngleHandle);
  xTaskCreate(TaskAltitudeSensor,  "Alt",   8192, NULL, 4, NULL);
  xTaskCreate(TaskCommunication,   "Comm",  4096, NULL, 3, NULL);
  xTaskCreate(TaskTelemetry,       "Tele",  8192, NULL, 2, NULL);
  xTaskCreate(TaskWebServer,       "Web",   8192, NULL, 1, NULL);

  Serial.println("[OK]  All tasks created — FLIGHT READY");
}

// loop() không dùng vì đã dùng RTOS
void loop() {
  vTaskDelete(NULL);
}

// ════════════════════════════════════════════════════════════
//  TASK 1 — RATE CONTROL (500Hz, Core 1, Priority 5)
//  Trách nhiệm:
//    - Đọc IMU (gyro + accel)
//    - Tính PID vòng trong (Rate)
//    - Ghi motor output
// ════════════════════════════════════════════════════════════
void TaskRateControl(void *pvParameters) {
  TickType_t xLastWake = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 1ms = 1KHz
  static bool yaw_armed = false;
  // Local copy của PID gains để tránh lock dài
  PidGains g_local;
  uint32_t last_us = micros();

  for (;;) {
    uint32_t now_us = micros();
    float dt_rate = (now_us - last_us) * 1e-6f;
    // Clamp dt phòng trường hợp bị delay bất thường
    dt_rate = constrain(dt_rate, 0.0005f, 0.01f);
    // Serial.println(dt_rate, 5);
    last_us = now_us;

    // ── 1. Đọc IMU qua I2C ───────────────────────────────
    ImuData imu_local;
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);

      imu_local.gx = (g.gyro.x - GX_OFgFlightET) * RAD_TO_DEG;
      imu_local.gy = (g.gyro.y - GY_OFgFlightET) * RAD_TO_DEG;
      imu_local.gz = -(g.gyro.z - GZ_OFgFlightET) * RAD_TO_DEG;
      imu_local.ax = (a.acceleration.x - AX_OFgFlightET);
      imu_local.ay = (a.acceleration.y - AY_OFgFlightET);
      imu_local.az = (a.acceleration.z - (AZ_OFgFlightET - GRAVITY));

      xSemaphoreGive(xI2CMutex);
    } else {
      // Không lấy được I2C — bỏ qua vòng này
      vTaskDelayUntil(&xLastWake, xPeriod);
      Serial.println("huhu");
      continue;
    }

    // ── 2. Publish IMU data cho AngleControl ─────────────
    if (xSemaphoreTake(xImuMutex, 0) == pdTRUE) {  // non-blocking
      imu_data = imu_local;
      xSemaphoreGive(xImuMutex);
    }

    // ── 3. Lấy target rates + throttle từ FlightState ────
    float tgt_rate_p, tgt_rate_r, tgt_rate_y;
    int thr_local;
    bool alt_hold_local;
    {
      // Đọc nhanh, không tính toán trong mutex
      xSemaphoreTake(xFlightMutex, portMAX_DELAY);
      tgt_rate_p = gFlight.target_rate_pitch;
      tgt_rate_r = gFlight.target_rate_roll;
      tgt_rate_y = gFlight.target_rate_yaw;
      thr_local = gFlight.throttle;
      alt_hold_local = gFlight.alt_hold;
      xSemaphoreGive(xFlightMutex);
    }
    // Serial.println(thr_local);
    // ── 4. Lấy PID gains ─────────────────────────────────
    if (xSemaphoreTake(xGainsMutex, 0) == pdTRUE) {
      g_local = gains;
      xSemaphoreGive(xGainsMutex);
    }

    // ── 5. Tính Rate PID (hoàn toàn local, không cần lock) ─
    // PITCH
    float err_p = tgt_rate_p - imu_local.gy;
    i_p += err_p * dt_rate;
    i_p = constrain(i_p, -200.0f, 200.0f);
    float pid_p_out = g_local.kp_p * err_p
                      + g_local.ki_p * i_p
                      + g_local.kd_p * (err_p - last_err_p) / dt_rate;
    pid_p_out = constrain(pid_p_out, -300.0f, 300.0f);
    last_err_p = err_p;

    // ROLL
    float err_r = tgt_rate_r - imu_local.gx;
    i_r += err_r * dt_rate;
    i_r = constrain(i_r, -200.0f, 200.0f);
    float pid_r_out = g_local.kp_r * err_r
                      + g_local.ki_r * i_r
                      + g_local.kd_r * (err_r - last_err_r) / dt_rate;
    pid_r_out = constrain(pid_r_out, -300.0f, 300.0f);
    last_err_r = err_r;

    // YAW
    float err_y = tgt_rate_y - imu_local.gz;
    i_yaw += err_y * dt_rate;
    i_yaw = constrain(i_yaw, -200.0f, 200.0f);
    float pid_yaw_out = g_local.kp_y * err_y
                        + g_local.ki_y * i_yaw
                        + g_local.kd_y * (err_y - last_err_yaw) / dt_rate;
    pid_yaw_out = constrain(pid_yaw_out, -300.0f, 300.0f);
    last_err_yaw = err_y;

    // ── 6. Publish PID outputs ───────────────────────────
    xSemaphoreTake(xFlightMutex, portMAX_DELAY);
    gFlight.pid_p = pid_p_out;
    gFlight.pid_r = pid_r_out;
    gFlight.pid_yaw = pid_yaw_out;
    xSemaphoreGive(xFlightMutex);

    // ── 7. Mixer + Motor output ──────────────────────────
    //
    //   M4(CCW)──front──M1(CW)
    //      \            /
    //      M3(CW)──M2(CCW)
    //
    //   Pitch+ (nose up)  → M1,M2 giảm / M3,M4 tăng  (+pid_p đẩy xuống)
    //   Roll+  (right)    → M1,M4 giảm / M2,M3 tăng  (+pid_r đẩy trái xuống)
    //   Yaw+   (CW)       → M1,M3 tăng / M2,M4 giảm  (CW motors tăng)
    //
    // int m1 = thr_local - (int)pid_p_out + (int)pid_r_out - (int)pid_yaw_out;
    // int m2 = thr_local - (int)pid_p_out - (int)pid_r_out + (int)pid_yaw_out;
    // int m3 = thr_local + (int)pid_p_out - (int)pid_r_out - (int)pid_yaw_out;
    // int m4 = thr_local + (int)pid_p_out + (int)pid_r_out + (int)pid_yaw_out;
    int m1 = thr_local - (int)pid_p_out;
    int m2 = thr_local - (int)pid_p_out;
    int m3 = thr_local + (int)pid_p_out;
    int m4 = thr_local + (int)pid_p_out;

    // int m1 = thr_local + (int)pid_r_out;
    // int m2 = thr_local - (int)pid_r_out;
    // int m3 = thr_local - (int)pid_r_out;
    // int m4 = thr_local + (int)pid_r_out;

    if (thr_local > 1030) {
      if (!yaw_armed) {
        xSemaphoreTake(xFlightMutex, portMAX_DELAY);
        gFlight.target_yaw = gFlight.yaw;   // lấy yaw từ QMC5883 đã tính trong TaskAngleControl
        yaw_armed = true;
        xSemaphoreGive(xFlightMutex);
      }
      writeMotor(0, m1);
      writeMotor(1, m2);
      writeMotor(2, m3);
      writeMotor(3, m4);
    } else {
      motorsOff();
      yaw_armed = false; 
      // Reset integrators khi throttle thấp (tránh integrator windup khi cầm tay)
      i_p = 0;
      i_r = 0;
      i_yaw = 0;
      last_err_p = 0;
      last_err_r = 0;
      last_err_yaw = 0;
    }
    // debug();
    // Serial.print(pitch)
    // Serial.println(dt_rate, 5);
    vTaskDelayUntil(&xLastWake, xPeriod);
  }
}

// ════════════════════════════════════════════════════════════
//  TASK 2 — ANGLE CONTROL (100Hz, Core 1, Priority 4)
//  Trách nhiệm:
//    - Lọc Kalman góc (pitch/roll)
//    - Đọc compass (yaw) — qua I2C mutex
//    - Tính PID vòng ngoài (Angle → Rate setpoint)
//    - Tính PID altitude (nếu alt_hold)
// ════════════════════════════════════════════════════════════
void TaskAngleControl(void *pvParameters) {
  TickType_t xLastWake = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(5);  // 5ms = 200Hz

  uint32_t last_us = micros();
  PidGains g_local;

  for (;;) {
    uint32_t now_us = micros();
    float dt_angle = (now_us - last_us) * 1e-6f;
    dt_angle = constrain(dt_angle, 0.005f, 0.05f);
    // Serial.println(dt_angle, 5);
    last_us = now_us;
  
    // ── 1. Lấy bản sao IMU data ──────────────────────────
    ImuData imu_local;
    xSemaphoreTake(xImuMutex, portMAX_DELAY);
    imu_local = imu_data;
    xSemaphoreGive(xImuMutex);

    // ── 2. Lấy gains ─────────────────────────────────────
    if (xSemaphoreTake(xGainsMutex, 0) == pdTRUE) {
      g_local = gains;
      xSemaphoreGive(xGainsMutex);
    }

    // ── 3. Kalman lọc góc (local vars, sau đó write vào gFlight) ─
    float pitch_acc = atan2f(-imu_local.ax,
                             sqrtf(imu_local.ay * imu_local.ay + imu_local.az * imu_local.az))
                      * RAD_TO_DEG;
    float roll_acc = atan2f(imu_local.ay,
                            sqrtf(imu_local.ax * imu_local.ax + imu_local.az * imu_local.az))
                     * RAD_TO_DEG;

    // Đọc pitch/roll + uncertainty hiện tại từ gFlight
    float pitch_k, roll_k, unc_pitch, unc_roll;
    xSemaphoreTake(xFlightMutex, portMAX_DELAY);
    pitch_k = gFlight.pitch;
    roll_k = gFlight.roll;
    unc_pitch = gFlight.KalmanUncertaintyAnglePitch;
    unc_roll = gFlight.KalmanUncertaintyAngleRoll;
    xSemaphoreGive(xFlightMutex);

    /* ------- COMPLEMENTARY ---------*/
    // float new_pitch = 0.98 * (pitch_k + imu_local.gy * dt_angle) + 0.02 * pitch_acc;
    // float new_roll = 0.98 * (roll_k + imu_local.gx * dt_angle) + 0.02 * roll_acc;

    kalman_1d(roll_k, unc_roll, imu_local.gx, roll_acc, dt_angle);
    float new_roll = Kalman1DOutput[0];
    float new_unc_r = Kalman1DOutput[1];

    kalman_1d(pitch_k, unc_pitch, imu_local.gy, pitch_acc, dt_angle);
    float new_pitch = Kalman1DOutput[0];
    float new_unc_p = Kalman1DOutput[1];

    // ── 4. Đọc Yaw từ compass (I2C) ──────────────────────
    float new_yaw = 0;
    static uint8_t yaw_div = 0;
    static float yaw_hold = 0.0f;
    yaw_div++;
    if (yaw_div >= 4) {  // Angle 200Hz / 4 = 50Hz
      yaw_div = 0;
      if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        sVector_t mag = compass.readRaw();
        compass.getHeadingDegrees();
        yaw_hold = mag.HeadingDegress;
        xSemaphoreGive(xI2CMutex);
      }
    }
    new_yaw = yaw_hold;

    // ── 5. Lấy targets từ FlightState ────────────────────
    float tgt_pitch, tgt_roll, tgt_yaw, tgt_alt;
    bool alt_hold_local, pos_hold_local;

    xSemaphoreTake(xFlightMutex, portMAX_DELAY);
    tgt_pitch = gFlight.target_pitch;
    tgt_roll = gFlight.target_roll;
    tgt_yaw = gFlight.target_yaw;
    tgt_alt = gFlight.target_alt;
    alt_hold_local = gFlight.alt_hold;
    pos_hold_local = gFlight.pos_hold;
    xSemaphoreGive(xFlightMutex);

    // Serial.pri
    // ── 6. GPS pos hold adjust ────────────────────────────
    float roll_adj = 0, pitch_adj = 0;
    if (pos_hold_local) {
      roll_adj = gps_roll_pid_adjust;
      pitch_adj = gps_pitch_pid_adjust;
    }

    float eff_target_pitch = tgt_pitch + pitch_adj;
    float eff_target_roll = tgt_roll + roll_adj;

    // ── 7. Angle PID ──────────────────────────────────────
    // PITCH
    float err_ap = eff_target_pitch - new_pitch;
    i_angle_p += err_ap * dt_angle;
    i_angle_p = constrain(i_angle_p, -80.0f, 80.0f);
    float rate_sp_pitch = constrain(g_local.kp_angle * err_ap
                                      + g_local.ki_angle * i_angle_p,
                                    -200.0f, 200.0f);

    // ROLL
    float err_ar = eff_target_roll - new_roll;
    i_angle_r += err_ar * dt_angle;
    i_angle_r = constrain(i_angle_r, -80.0f, 80.0f);
    float rate_sp_roll = constrain(g_local.kp_angle * err_ar
                                     + g_local.ki_angle * i_angle_r,
                                   -200.0f, 200.0f);
    
    // YAW (heading hold)
    float err_ay = tgt_yaw - new_yaw;
    if (err_ay > 180.0f) err_ay -= 360.0f;
    if (err_ay < -180.0f) err_ay += 360.0f;
    float rate_sp_yaw = constrain(g_local.kp_angle * err_ay, -200.0f, 200.0f);

    // ── 8. Altitude estimator + cascaded altitude PID ───────
    int thr_out = 0;  // delta throttle từ alt hold

    // Acc body -> Earth Z, đơn vị m/s^2.
    // Công thức này đúng hơn so với chỉ dùng az - cos(roll)*cos(pitch)*g.
    float roll_rad  = new_roll * DEG_TO_RAD;
    float pitch_rad = new_pitch * DEG_TO_RAD;
    float az_inertial = imu_local.ax * sinf(pitch_rad)
                      - imu_local.ay * sinf(roll_rad) * cosf(pitch_rad)
                      + imu_local.az * cosf(roll_rad) * cosf(pitch_rad)
                      - GRAVITY;

    // Kalman nên chạy liên tục, không chỉ khi bật alt_hold, để AltitudeKalman sẵn sàng khi chốt target.
    float cur_alt_cm = 0.0f;
    float R_meas = R_BMP;

    if (xSemaphoreTake(xAltMutex, 0) == pdTRUE) {
      cur_alt_cm = current_altitude;
      R_meas = altitude_meas_R_cm2;
      altitude_meas_new = false;
      xSemaphoreGive(xAltMutex);
    }

    kalman_2d(az_inertial, cur_alt_cm, dt_angle, R_meas);
    // Serial.println(cur_alt_cm);
    // Serial.println(AltitudeKalman);
    if (alt_hold_local) {
      // Vòng ngoài: độ cao -> target vertical velocity
      float alt_err = tgt_alt - AltitudeKalman;
      // Serial.println(alt_err);
      float target_vz = ALT_POS_KP * alt_err;
      target_vz = constrain(target_vz, -ALT_TARGET_VEL_LIMIT, ALT_TARGET_VEL_LIMIT);

      // Vòng trong: velocity -> throttle correction
      float vel_err = target_vz - VelocityVerticalKalman;
      float p_vel = g_local.kp_vel_z * vel_err;

      float i_vel = pre_vel_z_iterm + g_local.ki_vel_z * vel_err * dt_angle;
      i_vel = constrain(i_vel, -250.0f, 250.0f);

      // Derivative on measurement để tránh giật khi target_alt thay đổi.
      float d_meas = -(VelocityVerticalKalman - last_vz_for_d) / dt_angle;
      last_vz_for_d = VelocityVerticalKalman;
      float d_vel = g_local.kd_vel_z * d_meas;

      float pid_v = constrain(p_vel + i_vel + d_vel, -300.0f, 300.0f);

      pre_vel_z_err = vel_err;
      pre_vel_z_iterm = i_vel;

      // Bù mất lực nâng khi nghiêng. Giới hạn để tránh tăng ga quá mạnh.
      float thrust_comp = 1.0f / constrain(cosf(roll_rad) * cosf(pitch_rad), 0.75f, 1.0f);
      thrust_comp = constrain(thrust_comp, 1.0f, 1.30f);

      thr_out = (int)(pid_v * thrust_comp);
      // Serial.println(thr_out);
      // Publish acc_z & pid_vel để log
      xSemaphoreTake(xFlightMutex, portMAX_DELAY);
      gFlight.acc_z_inertial = az_inertial;
      gFlight.pid_vel = pid_v;
      xSemaphoreGive(xFlightMutex);
    } else {
      // Reset integrator altitude khi tắt alt hold
      pre_vel_z_err = 0;
      pre_vel_z_iterm = 0;
      last_vz_for_d = VelocityVerticalKalman;
    }

    // ── 9. Cập nhật throttle (nếu alt_hold) + targets vào gFlight ─
    xSemaphoreTake(xFlightMutex, portMAX_DELAY);
    gFlight.pitch = new_pitch;
    gFlight.roll = new_roll;
    gFlight.yaw = new_yaw;
    gFlight.pitch_acc = pitch_acc;
    gFlight.roll_acc = roll_acc;
    gFlight.AltitudeKalman = AltitudeKalman;
    gFlight.VelocityVerticalKalman = VelocityVerticalKalman;
    gFlight.KalmanUncertaintyAnglePitch = new_unc_p;
    gFlight.KalmanUncertaintyAngleRoll = new_unc_r;
    gFlight.target_rate_pitch = rate_sp_pitch;
    gFlight.target_rate_roll = rate_sp_roll;
    gFlight.target_rate_yaw = rate_sp_yaw;
    if (alt_hold_local) {
      gFlight.throttle = (int)BASE_HOVER_THROTTLE+ thr_out;
      gFlight.throttle = constrain(gFlight.throttle, 1000, 2000);
      Serial.println(BASE_HOVER_THROTTLE);
      Serial.println(thr_out);
    }
    xSemaphoreGive(xFlightMutex);

    // debug();
    vTaskDelayUntil(&xLastWake, xPeriod);
  }
}

// ════════════════════════════════════════════════════════════
//  TASK 3 — ALTITUDE SENSOR READ (50Hz, Core 0, Priority 4)
//  Trách nhiệm:
//    - Đọc BM280, TOF
// ════════════════════════════════════════════════════════════
void TaskAltitudeSensor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  float vz_local = 0.0;
  for (;;) {
    float bmp_cm = 0.0f;
    int tof_mm = -1;
    bool sensor_read_ok = false;

    // Chỉ khóa I2C khi đọc sensor
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bmp_cm = (bmp.readAltitude() - alt_ofgFlightet) * 100.0f;
      tof_mm = readTOF();
      xSemaphoreGive(xI2CMutex);
      sensor_read_ok = true;
      // Serial.println("hic");
    }
    // else
    //   Serial.println("not get i2cMutex");
    // Serial.println(tof_mm);
    if (sensor_read_ok) {
      bool tof_ok = false;
      float R_cm2 = R_BMP;

      float roll_local = 0.0f;
      float pitch_local = 0.0f;

      if (xSemaphoreTake(xFlightMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        roll_local = gFlight.roll;
        pitch_local = gFlight.pitch;
        vz_local = gFlight.VelocityVerticalKalman;
        xSemaphoreGive(xFlightMutex);
      }


      float alt_cm = processAltitudeMeasurement(
        tof_mm,
        roll_local,
        pitch_local,
        bmp_cm,
        vz_local,
        0.02f,
        tof_ok,
        R_cm2
      );
      // Serial.printf("tof_mm=%d bmp=%.1f alt=%.1f R=%.1f tof_ok=%d\n",
      //         tof_mm, bmp_cm, alt_cm, R_cm2, tof_ok);
      if (xSemaphoreTake(xAltMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        current_altitude = alt_cm;
        altitude_meas_R_cm2 = R_cm2;
        tof_valid = tof_ok;
        altitude_meas_new = true;
        xSemaphoreGive(xAltMutex);
      }
    }
    // debug();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ════════════════════════════════════════════════════════════
//  TASK 4 — COMMUNICATION (20Hz, Core 1, Priority 3)
//  Trách nhiệm:
//    - Đọc NRF24 (radio)
//    - Cập nhật targets + throttle vào FlightState
//    - Xử lý failsafe mất kết nối
// ════════════════════════════════════════════════════════════
void TaskCommunication(void *pvParameters) {
  ControlData rx_buf;

  for (;;) {
    bool got_packet = false;

    if (radio.available()) {
      radio.read(&rx_buf, sizeof(rx_buf));
      got_packet = true;
      timeout_connected = millis();
    }

    if (got_packet) {
      // Lấy gains để đọc danh_lai
      float dl;
      xSemaphoreTake(xGainsMutex, portMAX_DELAY);
      dl = gains.danh_lai;
      xSemaphoreGive(xGainsMutex);

      float new_target_pitch = mapf(rx_buf.trucX, 0, 1023, dl, -dl);
      float new_target_roll = mapf(rx_buf.trucY, 0, 1023, -dl, dl);
      // Serial.print(new_target_pitch);
      // Serial.print(',');
      // Serial.println(new_target_roll);
      xSemaphoreTake(xFlightMutex, portMAX_DELAY);

      gFlight.alt_hold = (bool)rx_buf.nut1;
      gFlight.pos_hold = (bool)rx_buf.nut2;

      // Khi bật alt_hold lần đầu → chốt target altitude
      static bool alt_init = false;
      if (gFlight.alt_hold && !gFlight.pos_hold) {
        if (!alt_init) {
          gFlight.target_alt = gFlight.AltitudeKalman;
          BASE_HOVER_THROTTLE = gFlight.throttle;
          pre_vel_z_err = 0;
          pre_vel_z_iterm = 0;
          last_vz_for_d = gFlight.VelocityVerticalKalman;
          alt_init = true;
        }
      } else {
        alt_init = false;
      }

      // Throttle chỉ cho remote control nếu KHÔNG alt hold
      if (!gFlight.alt_hold) {
        gFlight.throttle = (int)mapf(rx_buf.chinhtocdoquat, 0, 100, 1000, 1800);
      }

      gFlight.target_pitch = new_target_pitch;
      gFlight.target_roll = new_target_roll;

      // Cập nhật flight mode
      if (rx_buf.nut2 && !nut2_previous) {
        gFlight.flight_mode = (gFlight.flight_mode >= 3) ? 1 : 3;
      }
      nut2_previous = rx_buf.nut2;

      // Đặt target yaw hold = yaw hiện tại khi chưa có input yaw
      // (chỉ nếu joystick yaw ≈ center — mở rộng sau)

      xSemaphoreGive(xFlightMutex);
    }

    // ── Failsafe: mất liên lạc > 1 giây → hạ throttle ───
    if (millis() - timeout_connected > 1000) {
      if (millis() - time_throttle_drop > 400) {
        xSemaphoreTake(xFlightMutex, portMAX_DELAY);
        gFlight.alt_hold = false;
        if (gFlight.throttle > 1250) {
          gFlight.throttle -= 10;
        }
        xSemaphoreGive(xFlightMutex);
        time_throttle_drop = millis();
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz
  }
}


// ════════════════════════════════════════════════════════════
//  TOF + BMP altitude measurement processing
//  Output: altitude cm và R đo lường cm^2 để đưa vào Kalman
// ════════════════════════════════════════════════════════════
float processAltitudeMeasurement(int tof_mm,
                                 float roll_deg,
                                 float pitch_deg,
                                 float bmp_cm,
                                 float vz_cm_s,
                                 float dt,
                                 bool &tof_ok_out,
                                 float &R_out_cm2) {

  tof_ok_out = false;
  R_out_cm2 = R_BMP;

  if (tof_mm <= 0 || tof_mm > 9500) {
    return bmp_cm;
  }

  float tof_raw_cm = tof_mm * 0.1f;

  // Nếu nghiêng lớn, tia TOF nhìn lệch nhiều, không nên tin.
  if (fabsf(roll_deg) > 25.0f || fabsf(pitch_deg) > 25.0f) {
    return bmp_cm;
  }

  // Bù nghiêng: TOF đo đường xiên, cần chiếu về phương thẳng đứng.
  float roll_rad  = roll_deg * DEG_TO_RAD;
  float pitch_rad = pitch_deg * DEG_TO_RAD;
  float tof_cm = tof_raw_cm * cosf(roll_rad) * cosf(pitch_rad);

  if (!last_tof_ok) {
    last_tof_cm = tof_cm;
    last_tof_ok = true;
    tof_ok_out = true;
    R_out_cm2 = R_TOF_GOOD;
    return 0.85f * tof_cm + 0.15f * bmp_cm;
  }

  float delta = tof_cm - last_tof_cm;
  float rate = delta / constrain(dt, 0.02f, 0.2f);

  // TOF nhảy quá nhanh so với động học Z của drone: reject.
  if (fabsf(rate) > 1000.0f) {  // lech 1m 
    R_out_cm2 = R_BMP;
    return bmp_cm;
  }

  // TOF giảm mạnh nhưng velocity Z không cho thấy drone đang rơi nhanh:
  // khả năng cao là cây/cỏ/vật cản bên dưới.
  if (delta < -50.0f && vz_cm_s > -120.0f) {
    R_out_cm2 = R_BMP;
    return bmp_cm;
  }

  // TOF tăng mạnh: có thể gặp vùng trũng. Không bỏ hoàn toàn, nhưng giảm độ tin cậy.
  bool terrain_step_suspected = (delta > 80.0f && fabsf(vz_cm_s) < 120.0f);

  // Low-pass nhẹ cho TOF.
  float tof_filtered = 0.75f * last_tof_cm + 0.25f * tof_cm;    // cm
  last_tof_cm = tof_filtered;
  tof_ok_out = true;

  if (terrain_step_suspected) {
    R_out_cm2 = R_TOF_BAD;
    return 0.30f * tof_filtered + 0.70f * bmp_cm;
  }

  float w_tof = 0.0f;

  if (tof_filtered < 300.0f) {
    w_tof = 0.98f;
  } else if (tof_filtered < 800.0f) {
    w_tof = 0.95f;
  } else if (tof_filtered < 1000.0f) {
    float alpha = (tof_filtered - 800.0f) / 200.0f;
    alpha = constrain(alpha, 0.0f, 1.0f);
    w_tof = 0.95f + alpha * (0.50f - 0.95f);
  } else {
    w_tof = 0.0f;
  }

  float w_bmp = 1.0f - w_tof;

  R_out_cm2 =
    w_tof * w_tof * R_TOF_GOOD +
    w_bmp * w_bmp * R_BMP;

  return w_tof * tof_filtered + w_bmp * bmp_cm;
}

// ════════════════════════════════════════════════════════════
//  TASK 5 — TELEMETRY & SENSORS (10Hz, Core 0, Priority 2)
//  Trách nhiệm:
//    - Đọc INA219 (battery)
//    - Đọc GPS
//    - Ghi log
// ════════════════════════════════════════════════════════════
void TaskTelemetry(void *pvParameters) {
  for (;;) {

    // // ── INA219 battery ─────────────────────────────────
    // // INA219 cũng dùng I2C
    // if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    //   bat_voltage = INA219_read_voltage();
    //   xSemaphoreGive(xI2CMutex);
    // }

    // ── GPS ────────────────────────────────────────────
    // Sync bridge variables từ gFlight trước khi dùng
    // if (xSemaphoreTake(xFlightMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    //   acc_z_inertial = gFlight.acc_z_inertial;
    //   pid_vel = gFlight.pid_vel;
    //   throttle = gFlight.throttle;
    //   yaw = gFlight.yaw;
    //   xSemaphoreGive(xFlightMutex);
    // }
    // // GPS dùng UART riêng → không cần I2C mutex
    // readGPS(yaw);

    // // ── Log ────────────────────────────────────────────
    // if (log_flag) {
    //   logData();
    // }

    vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
  }
}

// ════════════════════════════════════════════════════════════
//  TASK 5 — WEB SERVER (Core 0, Priority 1)
//  Trách nhiệm:
//    - Serve trang PID tuning
//    - Nhận PID updates từ browser
// ════════════════════════════════════════════════════════════
void TaskWebServer(void *pvParameters) {
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  web_setup();
  Serial.print("[OK]  WiFi AP: ");
  Serial.println(WiFi.softAPIP());

  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ════════════════════════════════════════════════════════════
//  BMP280 SETUP — lấy ofgFlightet trung bình 100 mẫu
// ════════════════════════════════════════════════════════════
void bmp280_setup() {
  if (!bmp.begin(0x76)) {
    Serial.println("[ERR] BMP280 not found!");
    return;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);
  delay(500);

  double sum = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    sum += bmp.readAltitude();
    delay(10);
  }
  alt_ofgFlightet = (float)(sum / N);
  Serial.printf("[OK]  BMP280 — alt_ofgFlightet = %.2f m\n", alt_ofgFlightet);
}

// ════════════════════════════════════════════════════════════
//  HELPER: mapf (float version của map())
// ════════════════════════════════════════════════════════════
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void debug()
{
  float dbg_pitch, dbg_roll, tar_pitch, tar_roll;
  xSemaphoreTake(xFlightMutex, portMAX_DELAY);
  dbg_pitch = gFlight.pitch;
  dbg_roll  = gFlight.roll;
  tar_pitch = gFlight.target_pitch;
  tar_roll = gFlight.target_roll;
  xSemaphoreGive(xFlightMutex);
  Serial.printf("pitch=%.2f roll=%.2f tar_pitch=%.2f tar_roll=%.2f\n",
                dbg_pitch, dbg_roll, tar_pitch, tar_roll);
}