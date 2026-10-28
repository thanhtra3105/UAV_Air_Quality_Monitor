/*
 * types.h
 *
 *  Created on: Jun 27, 2026
 *      Author: lethanhtra
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32h5xx_hal.h"

/* ================= MOTOR CHANNELS ================= */
#define M1 TIM_CHANNEL_1
#define M2 TIM_CHANNEL_2
#define M3 TIM_CHANNEL_4
#define M4 TIM_CHANNEL_3


/* ================= RC & TELEMETRY PROTOCOL ================= */
#pragma pack(push, 1)
typedef struct {
	uint16_t trucX;
	uint16_t trucY;
	uint16_t trucZ;
	uint8_t trai;
	uint8_t phai;
	uint8_t len;
	uint8_t xuong;
	uint8_t batquathut;
	uint8_t nut1;
	uint8_t nut2;
	uint16_t chinhtocdoquat;
} ControlData;

typedef struct {
	int32_t lat;
	int32_t lon;
	float x;
	float y;
	float alt;
	float target_x;
	float target_y;
	float battery;
} TelemetryData;
#pragma pack(pop)

/* ================= FLIGHT STATES ================= */
typedef enum {
	ALT_STATE_IDLE = 0,
	ALT_STATE_TAKEOFF,
	ALT_STATE_HOLD,
	ALT_STATE_LANDING
} AltHoldState_t;

typedef enum {\
	MODE_ARM = 0,
	MODE_DISARM,
    MODE_MANUAL,
    MODE_TAKEOFF,
    MODE_POSHOLD,
    MODE_ALTHOLD,
    MODE_LANDING,
	MODE_NAVIGATION
} FlightMode_t;

/* ================= MATH UTILITIES ================= */
static inline float constrain(float data, float min, float max) {
	if (data <= min)
		return min;
	if (data >= max)
		return max;
	return data;
}

static inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif /* INC_TYPES_H_ */
