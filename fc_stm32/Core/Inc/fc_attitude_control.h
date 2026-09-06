/**
 * @file fc_attitude_control.h
 * @brief Cascaded Attitude & Angular Rate PID Controller
 */

#ifndef FC_ATTITUDE_CONTROL_H_
#define FC_ATTITUDE_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"
#include "pid_controller.h"

#define MAX_ALT_OUTPUT 200.0f

extern PIDController_t PID_Angle;
extern PIDController_t PID_Rate_Pitch;
extern PIDController_t PID_Rate_Roll;
extern PIDController_t PID_Rate_Yaw;

void AttitudeControl_Init(void);
void AttitudeControl_AngleLoop(VehicleState_t *veh, float dt);
void AttitudeControl_RateLoop(VehicleState_t *veh, float dt);
void AttitudeControl_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_ATTITUDE_CONTROL_H_ */
