/**
 * @file fc_core.h
 * @brief Flight Controller Core & Main Loop Scheduler (AP_Scheduler style)
 */

#ifndef FC_CORE_H_
#define FC_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"
#include "fc_motors.h"
#include "fc_rc.h"
#include "fc_telemetry.h"
#include "fc_ahrs.h"
#include "fc_pos_estimator.h"
#include "fc_attitude_control.h"
#include "fc_flight_modes.h"

void FlightController_Init(void);
void FlightController_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_CORE_H_ */
