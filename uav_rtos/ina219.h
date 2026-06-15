#pragma once
#include <Wire.h>

// INA219 @ 0x40 — đọc bus voltage (register 0x02)
// QUAN TRỌNG: Caller phải lấy xI2CMutex trước khi gọi hàm này
float INA219_read_voltage() {
  Wire.beginTransmission(0x40);
  Wire.write(0x02);
  if (Wire.endTransmission() != 0) return -1.0f;  // lỗi I2C
  Wire.requestFrom(0x40, 2);
  if (Wire.available() < 2) return -1.0f;
  uint16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw >> 3) * 0.004f;  // 4mV per LSB
}