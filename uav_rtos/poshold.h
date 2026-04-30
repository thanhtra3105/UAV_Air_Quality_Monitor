#pragma once
#include <stdint.h>
#include <stdbool.h>



// ── API ────────────────────────────────────────────────────
void gpsSetup();
void readGPS(float);
bool checkGPSQuality();
void gps_debug();