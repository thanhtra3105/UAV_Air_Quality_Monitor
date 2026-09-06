/**
 * @file fc_pos_estimator.h
 * @brief Position and Velocity Estimator (GPS & Optical Flow Fusion)
 */

#ifndef FC_POS_ESTIMATOR_H_
#define FC_POS_ESTIMATOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"
#include "kalman_gps.h"
#include "kalman.h"
#include "mtf01.h"
#include "gps.h"

extern KalmanFilter1D_t kf_North;
extern KalmanFilter1D_t kf_East;
extern KalmanAxis_t     kf_x;
extern KalmanAxis_t     kf_y;
extern MTF01_t          mtf_data;
extern GPS_Data_t       gps;

void PosEstimator_Init(void);
void PosEstimator_UpdateGPS(VehicleState_t *veh, float dt);
void PosEstimator_UpdateOpticalFlow(VehicleState_t *veh, float dt);

#ifdef __cplusplus
}
#endif

#endif /* FC_POS_ESTIMATOR_H_ */
