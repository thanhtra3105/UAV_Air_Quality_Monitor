#include "poshold.h"
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <math.h>
#include <Arduino.h>

// ════════════════════════════════════════════════════════════
//  CONFIG GPS UART
// ════════════════════════════════════════════════════════════
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

/*
  uBlox M10 config commands (gửi 1 lần qua Serial hoặc u-center):
  Set 10 Hz : B5 62 06 8A 0A 00 00 07 00 00 01 00 21 30 64 00 57 EF
  En GGA+RMC: B5 62 06 8A 22 00 00 07 00 00 BB 00 91 20 01
              AC 00 91 20 01 CA 00 91 20 00 C0 00 91 20 00
              C5 00 91 20 00 B1 00 91 20 00 48 26
*/

static TinyGPSPlus gps;
static HardwareSerial SerialGPS(1);

// ════════════════════════════════════════════════════════════
//  GPS GLOBALS  (extern trong poshold.h)
// ════════════════════════════════════════════════════════════
float current_lat = 0.0f;
float current_lon = 0.0f;
float current_speed_ms = 0.0f;
float gpsHeading = 0.0f;
float hdop = 99.0f;
uint8_t satellites = 0;

float hold_lat = 0.0f;
float hold_lon = 0.0f;

// ════════════════════════════════════════════════════════════
//  EXTERN TỪ MAIN
// ════════════════════════════════════════════════════════════
extern float pitch, roll, yaw;  // độ
extern float ax, ay, az;        // m/s² (raw từ MPU6050)
extern float target_pitch, target_roll;
extern float dt;  // dt main loop (giây)

// ════════════════════════════════════════════════════════════
//  PID GAINS  (tune ở đây)
// ════════════════════════════════════════════════════════════
float kp_pos = 0.8f;     // vị trí → vận tốc mục tiêu
float kp_vel_xy = 0.9f;  // vận tốc → góc nghiêng
float ki_vel_xy = 0.01f;
float kd_vel_xy = 0.05f;

// ════════════════════════════════════════════════════════════
//  INTERNAL STATE
// ════════════════════════════════════════════════════════════
static float i_vel_x = 0.0f, i_vel_y = 0.0f;
static float last_err_vel_x = 0.0f, last_err_vel_y = 0.0f;
static float dt_gps = 0.1f;
static uint32_t last_gps_ms = 0;
static bool gps_new_data = false;

// ── Kalman 1D ────────────────────────────────────────────────
// State: velocity (m/s) theo 1 trục
// Predict @ 200 Hz bằng IMU accel
// Update  @ 10 Hz  bằng GPS velocity (COG + speed)
// ─────────────────────────────────────────────────────────────
// Tune:
//   KF_Q lớn → không tin IMU nhiều → output bám GPS hơn (lag hơn)
//   KF_R lớn → không tin GPS nhiều → output mượt hơn nhưng chậm correct
#define KF_Q 0.5f  // process noise (MPU6050 drift khá)
#define KF_R 0.8f  // measurement noise (GPS COG không chính xác tốc độ thấp)

typedef struct {
  float v;
  float P;
} KF1D_t;
static KF1D_t kf_vx = { 0.0f, 1.0f };  // North (m/s)
static KF1D_t kf_vy = { 0.0f, 1.0f };  // East  (m/s)

static void kf_predict(KF1D_t *kf, float accel, float _dt) {
  kf->v += accel * _dt;  // tích phân gia tốc
  kf->P += KF_Q * _dt;   // uncertainty tăng
}

static void kf_update(KF1D_t *kf, float gps_v) {
  float K = kf->P / (kf->P + KF_R);
  kf->v += K * (gps_v - kf->v);  // correct về GPS
  kf->P *= (1.0f - K);
  if (kf->P < 0.01f) kf->P = 0.01f;  // tránh P → 0
}

// ════════════════════════════════════════════════════════════
//  ROTATE BODY ACCEL → NED  (chỉ trục ngang X,Y)
// ════════════════════════════════════════════════════════════
//  Input : ax_b, ay_b, az_b  [m/s²]  body frame (MPU6050 raw)
//          pitch_deg, roll_deg        từ complementary filter
//  Output: ax_north, ay_east [m/s²]  NED, gravity đã bị bù tự nhiên
//
//  Tại sao gravity tự bù?
//    Khi level: az_b ≈ +9.81, pitch=roll=0 → ax_north ≈ 0  ✓
//    Khi pitch 10° về Bắc: az_b vẫn ≈ 9.81 nhưng
//      ax_north = cp*cr*ax_b + cp*sr*ay_b - sp*az_b
//              ≈ -sin(10°)*9.81 ≈ -1.7 m/s²  ← gia tốc ngang thật về Bắc
// ────────────────────────────────────────────────────────────
static void rotateAccToNED(float ax_b, float ay_b, float az_b,
                           float pitch_deg, float roll_deg,
                           float *ax_north, float *ay_east) {
  float p = pitch_deg * (3.14159f / 180.0f);
  float r = roll_deg * (3.14159f / 180.0f);
  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);

  *ax_north = cp * cr * ax_b + cp * sr * ay_b - sp * az_b;
  *ay_east = cr * ay_b - sr * az_b;
  // Lưu ý: gravity (az_b≈9.81 khi level) contribute:
  //   ax_north += -sp * 9.81  → đúng với gia tốc ngang North khi pitch
  //   ay_east  += -sr * 9.81  → đúng với gia tốc ngang East khi roll
  // Không cần trừ thêm gravity riêng.
}

// ════════════════════════════════════════════════════════════
//  PUBLIC API
// ════════════════════════════════════════════════════════════

void gpsSetup() {
  SerialGPS.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void readGPS() {
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    if (gps.encode(c)) {
      if (gps.location.isValid() && gps.location.isUpdated()) {
        current_lat = (float)gps.location.lat();
        current_lon = (float)gps.location.lng();
        current_speed_ms = gps.speed.mps();
        satellites = gps.satellites.value();
        hdop = gps.hdop.hdop();
        gpsHeading = gps.course.deg();
        // Serial.print("current_lat: ");
        // Serial.println(current_lat);
        poshold_update_gps();  // update Kalman ngay khi có fix
      }
    }
  }
}

bool checkGPSQuality() {
  if (hdop > 2.0f) return false;     // HDOP cao = nhiễu nhiều
  if (satellites < 8) return false;  // ít vệ tinh = không tin
  return true;
}

// Gọi mỗi lần đọc IMU trong calculateAngle() — ~200 Hz
void poshold_predict_imu() {
  float ax_north, ay_east;
  rotateAccToNED(ax, ay, az, pitch, roll, &ax_north, &ay_east);
  kf_predict(&kf_vx, ax_north, dt);
  kf_predict(&kf_vy, ay_east, dt);
}

// Gọi bên trong readGPS() khi có location fix mới — ~10 Hz
void poshold_update_gps() {
  uint32_t now = millis();
  dt_gps = constrain((now - last_gps_ms) / 1000.0f, 0.05f, 0.3f);
  last_gps_ms = now;
  gps_new_data = true;

  if (current_speed_ms >= 0.25f) {
    // COG đáng tin khi đang di chuyển
    float cog_rad = gpsHeading * (3.14159f / 180.0f);
    kf_update(&kf_vx, current_speed_ms * cosf(cog_rad));  // North
    kf_update(&kf_vy, current_speed_ms * sinf(cog_rad));  // East
  } else {
    // Gần đứng yên — kéo nhẹ về 0, COG không tin được
    kf_update(&kf_vx, 0.0f);
    kf_update(&kf_vy, 0.0f);
  }
}

// Gọi trong loop() — chỉ chạy khi có GPS fix mới
void calculatePosHold() {
  if (!gps_new_data) return;
  if (!checkGPSQuality()) {
    gps_new_data = false;
    return;
  }
  gps_new_data = false;

  // ── 1. Lỗi vị trí → mét (NED) ──────────────────────────
  float dist_north = (hold_lat - current_lat) * 111320.0f;
  float dist_east = (hold_lon - current_lon) * 111320.0f
                    * cosf(current_lat * (3.14159f / 180.0f));

  // ── 2. P vị trí → vận tốc mục tiêu ─────────────────────
  float target_vx = constrain(kp_pos * dist_north, -2.0f, 2.0f);
  float target_vy = constrain(kp_pos * dist_east, -2.0f, 2.0f);

  // ── 3. Velocity thực từ Kalman (mượt @ 200 Hz) ──────────
  float actual_vx = kf_vx.v;
  float actual_vy = kf_vy.v;

  // ── 4. PID vận tốc → góc nghiêng Earth frame ────────────
  float err_vx = target_vx - actual_vx;
  float err_vy = target_vy - actual_vy;

  i_vel_x += err_vx * dt_gps;
  i_vel_y += err_vy * dt_gps;
  i_vel_x = constrain(i_vel_x, -10.0f, 10.0f);
  i_vel_y = constrain(i_vel_y, -10.0f, 10.0f);

  float d_vx = (err_vx - last_err_vel_x) / dt_gps;
  float d_vy = (err_vy - last_err_vel_y) / dt_gps;
  last_err_vel_x = err_vx;
  last_err_vel_y = err_vy;

  float tilt_north = kp_vel_xy * err_vx + ki_vel_xy * i_vel_x + kd_vel_xy * d_vx;
  float tilt_east = kp_vel_xy * err_vy + ki_vel_xy * i_vel_y + kd_vel_xy * d_vy;
  tilt_north = constrain(tilt_north, -15.0f, 15.0f);
  tilt_east = constrain(tilt_east, -15.0f, 15.0f);

  // ── 5. Rotate Earth → Body (dùng yaw la bàn) ────────────
  float cy = cosf(yaw * (3.14159f / 180.0f));
  float sy = sinf(yaw * (3.14159f / 180.0f));

  // Thêm vào cuối calculatePosHold(), trước dòng target_pitch = ...
  Serial.print("dist_N: ");
  Serial.print(dist_north, 3);
  Serial.print("  dist_E: ");
  Serial.print(dist_east, 3);
  Serial.print("  vx: ");
  Serial.print(actual_vx, 3);
  Serial.print("  vy: ");
  Serial.print(actual_vy, 3);
  Serial.print("  tilt_N: ");
  Serial.print(tilt_north, 3);
  Serial.print("  tilt_E: ");
  Serial.println(tilt_east, 3);

  target_pitch = -(tilt_north * cy + tilt_east * sy);
  target_roll = (tilt_east * cy - tilt_north * sy);
}