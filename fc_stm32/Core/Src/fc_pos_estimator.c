/**
 * @file fc_pos_estimator.c
 * @brief Position and Velocity Estimator Implementation
 */

#include "fc_pos_estimator.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

KalmanFilter1D_t kf_North;
KalmanFilter1D_t kf_East;
KalmanAxis_t     kf_x;
KalmanAxis_t     kf_y;
MTF01_t          mtf_data;
GPS_Data_t       gps;

#define OPTICAL_FLOW_R 0.5f
#define KI_FLOW        0.02f

void PosEstimator_Init(void) {
    GPS_Init_DMA(&huart1);
    MTF01_Init(&huart2);

    KalmanGPS_Init(&kf_North, 0.0f, 0.0f, 0.1f, 0.001f);
    KalmanGPS_Init(&kf_East,  0.0f, 0.0f, 0.1f, 0.001f);

    KalmanAxis_Init(&kf_x);
    KalmanAxis_Init(&kf_y);
}

void PosEstimator_UpdateGPS(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    /*==========================================================================
     * 1. Rotate Accelerations from Body to Earth Frame (FRD -> NED)
     *==========================================================================*/
    float ax_frd =  veh->ax;
    float ay_frd = -veh->ay;
    float az_frd = -veh->az;

    float cy = cosf(veh->yaw * DEG_TO_RAD);
    float sy = sinf(veh->yaw * DEG_TO_RAD);
    float cp = cosf(veh->pitch * DEG_TO_RAD);
    float sp = sinf(veh->pitch * DEG_TO_RAD);
    float cr = cosf(veh->roll * DEG_TO_RAD);
    float sr = sinf(veh->roll * DEG_TO_RAD);

    veh->ax_earth = cy * cp * ax_frd + (-cy * sp * sr - sy * cr) * ay_frd
                  + (-cy * sp * cr + sy * sr) * az_frd;

    veh->ay_earth = sy * cp * ax_frd + (-sy * sp * sr + cy * cr) * ay_frd
                  + (-sy * sp * cr - cy * sr) * az_frd;

    // Convert g to m/s^2
    veh->ax_earth *= 9.81f;
    veh->ay_earth *= 9.81f;

    /*==========================================================================
     * 2. Kalman PREDICT (High rate, e.g. 500Hz from IMU)
     *==========================================================================*/
    KalmanGPS_Predict(&kf_North, veh->ax_earth, dt);
    KalmanGPS_Predict(&kf_East,  veh->ay_earth, dt);

    /*==========================================================================
     * 3. Kalman UPDATE (~10Hz when GPS packet arrives)
     *==========================================================================*/
    if (gps.fixType >= 3 && gps.ready == 1) {
        if (!veh->gps_home_set) {
            veh->home_lat = gps.latitude;
            veh->home_lon = gps.longitude;
            veh->gps_home_set = true;

            kf_North.pos = 0.0f;
            kf_North.vel = 0.0f;
            kf_North.bias = 0.0f;
            kf_East.pos  = 0.0f;
            kf_East.vel  = 0.0f;
            kf_East.bias = 0.0f;
        } else {
            float lat_err = (float)(gps.latitude - veh->home_lat);
            float lon_err = (float)(gps.longitude - veh->home_lon);

            float pos_N = lat_err * 111320.0f;
            float pos_E = lon_err * 111320.0f * cosf((float)veh->home_lat * DEG_TO_RAD);

            float vel_N = (float)gps.velN / 1000.0f;
            float vel_E = (float)gps.velE / 1000.0f;

            float r_pos_noise = ((float)gps.hAcc / 1000.0f) * ((float)gps.hAcc / 1000.0f);
            float r_vel_noise = ((float)gps.sAcc / 1000.0f) * ((float)gps.sAcc / 1000.0f);

            KalmanGPS_Update(&kf_North, pos_N, vel_N, r_pos_noise, r_vel_noise);
            KalmanGPS_Update(&kf_East,  pos_E, vel_E, r_pos_noise, r_vel_noise);
        }
        gps.ready = 0;
    }

    /*==========================================================================
     * 4. Assign states to VehicleState
     *==========================================================================*/
    veh->est_x  = kf_North.pos;
    veh->est_vx = kf_North.vel;
    veh->est_y  = kf_East.pos;
    veh->est_vy = kf_East.vel;
}

void PosEstimator_UpdateOpticalFlow(VehicleState_t *veh, float dt) {
    if (veh == NULL) return;

    float cy = cosf(veh->yaw * DEG_TO_RAD);
    float sy = sinf(veh->yaw * DEG_TO_RAD);
    float cp = cosf(veh->pitch * DEG_TO_RAD);
    float sp = sinf(veh->pitch * DEG_TO_RAD);
    float cr = cosf(veh->roll * DEG_TO_RAD);
    float sr = sinf(veh->roll * DEG_TO_RAD);

    veh->ax_earth = cy * cp * veh->ax + (cy * sp * sr - sy * cr) * veh->ay
                  + (cy * sp * cr + sy * sr) * veh->az;

    veh->ay_earth = sy * cp * veh->ax + (sy * sp * sr + cy * cr) * veh->ay
                  + (sy * sp * cr - cy * sr) * veh->az;

    veh->ax_earth *= 9.81f;
    veh->ay_earth *= 9.81f;

    float body_vx = -mtf_data.vy;
    float body_vy =  mtf_data.vx;

    float flow_vx = body_vx * cy - body_vy * sy;
    float flow_vy = body_vx * sy + body_vy * cy;

    float ax_input = veh->ax_earth - veh->ax_bias;
    float ay_input = veh->ay_earth - veh->ay_bias;

    KalmanAxis_Predict(&kf_x, ax_input, dt);
    KalmanAxis_Predict(&kf_y, ay_input, dt);

    if (mtf_data.flow_quality > 50 && mtf_data.distance >= 100) {
        KalmanAxis_UpdateVel(&kf_x, flow_vx, OPTICAL_FLOW_R);
        KalmanAxis_UpdateVel(&kf_y, flow_vy, OPTICAL_FLOW_R);

        float err_vx = flow_vx - kf_x.vel;
        float err_vy = flow_vy - kf_y.vel;
        veh->ax_bias += KI_FLOW * err_vx * dt;
        veh->ay_bias += KI_FLOW * err_vy * dt;
    }

    veh->est_x  = kf_x.pos;
    veh->est_vx = kf_x.vel;
    veh->est_y  = kf_y.pos;
    veh->est_vy = kf_y.vel;
}
