/**
 * @file fc_rc.c
 * @brief RC Receiver (NRF24) and Failsafe Implementation
 */

#include "fc_rc.h"
#include "nrf.h"
#include "gps.h"
#include "serial.h"

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern GPS_Data_t gps;

static uint8_t nrf_address[5] = { '0', '0', '0', '0', '1' };

void RC_Init(void) {
	HAL_SPI_Init(&hspi2);

	NRF_CSN_HIGH();
	HAL_Delay(10);

	NRF24_Init();
	NRF24_OpenReadingPipe(0, nrf_address);
	NRF24_StartListening();
	HAL_Delay(500);
}

void RC_WaitForZeroThrottle(VehicleState_t *veh) {
	Serial_printf(&huart1, "[WAITING] Waiting pull throttle to 0!\r\n");
	while (1) {
		if (NRF24_Available()) {
			uint8_t len = NRF24_GetDynamicPayloadSize();
			if (len == sizeof(ControlData)) {
				NRF24_Read((uint8_t*) &veh->rc_raw, len);
				if (veh->rc_raw.chinhtocdoquat == 1000) {
					veh->rc_throttle = 1000;
					veh->throttle = 1000;
					break;
				}
			}
		}
	}
}

void RC_Failsafe(VehicleState_t *veh) {
	uint32_t now = HAL_GetTick();

	if (now - veh->last_rc_received_tick > 1000) {
		veh->failsafe_active = true;
		veh->alt_hold_active = false;

		if (now - veh->last_failsafe_throttle_tick > 400) {
			if (veh->throttle > 1200) {
				veh->throttle -= 5;
				veh->last_failsafe_throttle_tick = now;
			} else {
				// When descending near ground / slow descent
				if (veh->est_vz < 20.0f && veh->alt < 20) {
					veh->throttle = 1000;	// disarm
				}
			}
		}
	} else {
		veh->failsafe_active = false;
	}
}
void RC_Process(VehicleState_t *veh) {
	if (veh == NULL)
		return;

	if (NRF24_Available()) {
		uint8_t len_nrf = NRF24_GetDynamicPayloadSize();
		if (len_nrf == sizeof(ControlData)) {
			NRF24_Read((uint8_t*) &veh->rc_raw, len_nrf);

			// Chỉ đọc và lưu các tín hiệu từ tay cầm
			veh->rc_throttle = veh->rc_raw.chinhtocdoquat;
			veh->rc_target_roll = (float) map_val(veh->rc_raw.trucX, 0, 100,
					(long) MAX_TARGET_ROLL_PITCH,
					(long) -MAX_TARGET_ROLL_PITCH);
			veh->rc_target_pitch = (float) map_val(veh->rc_raw.trucY, 0, 100,
					(long) MAX_TARGET_ROLL_PITCH,
					(long) -MAX_TARGET_ROLL_PITCH);

			veh->last_rc_received_tick = HAL_GetTick();
		}

		// Gửi telemetry phản hồi qua ACK Payload
		veh->telemetry.lat = (int32_t) (gps.latitude * 1e7);
		veh->telemetry.lon = (int32_t) (gps.longitude * 1e7);
		veh->telemetry.x = veh->est_x;
		veh->telemetry.y = veh->est_y;
		veh->telemetry.alt = veh->est_alt;
		veh->telemetry.battery = veh->battery_voltage;

		NRF24_WriteAckPayload(0, &veh->telemetry, sizeof(TelemetryData));

	} else {
		RC_Failsafe(veh);
	}
}

