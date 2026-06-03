/* GY-TOF10 with I2C*/
#pragma once
#include <Wire.h>

#define TOF_ADDR 0xA4 >> 1
int readTOF() {
  Wire.beginTransmission(TOF_ADDR);
  Wire.write(0x08);  // distance high reg
  if (Wire.endTransmission(false) != 0)
    return -1;

  if (Wire.requestFrom(TOF_ADDR, 2) != 2)
    return -1;
  uint8_t dis_h = Wire.read();
  uint8_t dis_l = Wire.read();
  return ((dis_h << 8) | dis_l);  // mm
}