#pragma once

class PIDController {
private:
  float kp, ki, kd;
  float preE;
  float preI;
  
  // Khai báo thêm biến cho bộ lọc D
  float preD;   
  float alphaD; // Hệ số lọc LPF (từ 0.0 đến 1.0)

public:
  // Thêm tham số alpha vào hàm khởi tạo. Mặc định là 0.5 (Lọc khá mạnh)
  PIDController(float p, float i, float d, float alpha = 0.5f) {
    kp = p;
    ki = i;
    kd = d;
    preE = 0.0f;
    preI = 0.0f;
    preD = 0.0f;
    alphaD = alpha; // Giá trị càng nhỏ lọc càng mạnh, nhưng độ trễ càng cao
  }

  float calculate(float err, float dt) {
    if (dt <= 0) return 0;
    
    // 1. Khâu P
    float P = kp * err;
    
    // 2. Khâu I
    preI += ki * (err + preE) * dt / 2.0f;
    preI = constrain(preI, -200.0f, 200.0f); 
    float I = preI;

    // 3. Khâu D (Đã thêm bộ lọc LPF)
    float rawD = kd * (err - preE) / dt;
    
    // Công thức lọc LPF: Lấy một phần giá trị mới cộng với phần lớn giá trị cũ
    float D = alphaD * rawD + (1.0f - alphaD) * preD;

    // 4. Cập nhật biến trạng thái
    preE = err;
    preD = D; // Lưu lại khâu D đã lọc cho vòng lặp sau

    return (P + I + D);
  }

  void reset() {
    preE = 0.0f;
    preI = 0.0f;
    preD = 0.0f; // Reset cả khâu D
  }

  void set(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }
};