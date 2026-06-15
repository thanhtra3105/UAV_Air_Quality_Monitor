#pragma once

#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.017453f

const float R_TOF_GOOD = 25.0f;     // std ≈ 5cm
const float R_TOF_BAD  = 900.0f;    // std ≈ 30cm
const float R_BMP      = 2500.0f;   // std ≈ 50cm

// Trạng thái lưu TOF lần trước
static float last_tof_cm = 0.0f;
static bool  last_tof_ok = false;

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