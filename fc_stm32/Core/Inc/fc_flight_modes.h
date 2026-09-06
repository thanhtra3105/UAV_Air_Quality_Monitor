/**
 * @file fc_flight_modes.h
 * @brief Flight Modes & Position / Navigation Controller
 */

#ifndef FC_FLIGHT_MODES_H_
#define FC_FLIGHT_MODES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"
#include "pid_controller.h"
#include "mission.h"

#define TAKEOFF_TARGET_ALT   500.0f   // cm
#define TAKEOFF_SPEED        100.0f   // cm/s
#define LANDING_SPEED        40.0f    // cm/s
#define HOVER_THROTTLE_BASE  1400.0f
#define WAYPOINT_RADIUS      2.0f     // meters
#define MAX_NAV_SPEED        5.0f     // m/s
#define MAX_TARGET_VZ        80.0f    // cm/s

extern PIDController_t PID_Alt_Pos;
extern PIDController_t PID_Alt_Vel;
extern PIDController_t PID_Pos_X;
extern PIDController_t PID_Pos_Y;
extern PIDController_t PID_Vel_X;
extern PIDController_t PID_Vel_Y;

void FlightModes_Init(void);
void FlightModes_Update(VehicleState_t *veh, float dt);

void Mode_AltHold_Update(VehicleState_t *veh, float dt);
void Mode_GPS_PositionHold_Update(VehicleState_t *veh, float dt);
void Mode_OpticalFlow_Hold_Update(VehicleState_t *veh, float dt);
void Mode_AutoTakeoff(VehicleState_t *veh, float dt);
void Mode_AutoLanding(VehicleState_t *veh, float dt);
void Run_XY_Controller(VehicleState_t *veh, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_FLIGHT_MODES_H_ */
