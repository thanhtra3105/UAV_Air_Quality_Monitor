/**
 * @file fc_telemetry.h
 * @brief Telemetry & Ground Control Station Communication
 */

#ifndef FC_TELEMETRY_H_
#define FC_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"

void Telemetry_Init(void);
void Telemetry_UpdateBattery(VehicleState_t *veh);
void Telemetry_SendESP32(VehicleState_t *veh);
void Telemetry_UpdateLEDs(const VehicleState_t *veh);

#ifdef __cplusplus
}
#endif

#endif /* FC_TELEMETRY_H_ */
