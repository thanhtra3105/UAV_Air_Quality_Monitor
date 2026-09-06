/**
 * @file fc_ahrs.h
 * @brief Attitude and Heading Reference System (AHRS) & Sensor Processing
 */

#ifndef FC_AHRS_H_
#define FC_AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"
#include "icm20602.h"
#include "dsp310.h"
#include "ist8310.h"
#include "qmc5883.h"
#include "kalman.h"

// Choose magnetometer
#define USE_IST8310
//#define USE_QMC5883

extern ICM20602_t    imu;
extern DSP310_t      dsp_sensor;
extern IST8310_Data_t ist8310;
extern Kalman4D_t    kf_4d;

void  AHRS_Init(VehicleState_t *veh);
void  AHRS_Calibrate(VehicleState_t *veh);
void  AHRS_ReadIMU(VehicleState_t *veh);
float AHRS_ReadHeading(float roll_deg, float pitch_deg);
void  AHRS_UpdateAttitude(VehicleState_t *veh, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_AHRS_H_ */
