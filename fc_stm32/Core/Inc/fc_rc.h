/**
 * @file fc_rc.h
 * @brief RC Receiver (NRF24) and Failsafe Handler
 */

#ifndef FC_RC_H_
#define FC_RC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"

#define MAX_TARGET_ROLL_PITCH 8.0f

void RC_Init(void);
void RC_WaitForZeroThrottle(VehicleState_t *veh);
void RC_Process(VehicleState_t *veh);
void RC_Failsafe(VehicleState_t *veh);

#ifdef __cplusplus
}
#endif

#endif /* FC_RC_H_ */
