///*
// * pos_hold.h
// *
// *  Created on: Jul 4, 2026
// *      Author: lethanhtra
// */
//
//#ifndef INC_POS_HOLD_H_
//#define INC_POS_HOLD_H_
//
//#include "stm32h5xx_hal.h"
//#include <stdint.h>
//#include "mtf01.h"
//#include "pid_controller.h"
//#include "kalman.h"
//
///* ================= Cau hinh chung ================= */
//#define POS_HOLD_MAX_TILT_DEG      12.0f      // gioi han goc nghieng toi da khi giu vi tri (do)
//#define MTF01_QUALITY_MIN          30         // duoi nguong nay khong tin tuong flow
//#define MTF01_HEIGHT_MIN           0.08f      // m, duoi do flow khong hop le
//#define MTF01_HEIGHT_MAX           8.0f       // m, tren do (tuy ban dung MTF-01 hay 01P, 01P toi 12m nhung optical flow van gioi han theo datasheet PMW3901)
//#define POS_HOLD_LOWQ_TIMEOUT_MS   1000       // neu mat flow lien tuc qua thoi gian nay -> bao "khong tin cay"
//
//
//typedef enum {
//    POS_HOLD_DISABLED = 0,
//    POS_HOLD_ENABLED  = 1
//} PosHold_State_t;
//
//typedef struct {
//    KalmanAxis_t kf_x;   // truc "North" (world frame, tuong doi)
//    KalmanAxis_t kf_y;   // truc "East"  (world frame, tuong doi)
//
//    PIDController_t pid_pos_x;
//    PIDController_t pid_pos_y;
//    PIDController_t pid_vel_x;
//    PIDController_t pid_vel_y;
//
//    float target_x;  // vi tri muon giu (m, world frame)
//    float target_y;
//
//    float roll_sp;    // rad - OUTPUT, cong them vao attitude controller hien co
//    float pitch_sp;   // rad - OUTPUT
//
//    PosHold_State_t state;
//
//    uint32_t low_quality_ms;  // thoi gian lien tuc flow khong dang tin (ms)
//    uint8_t  is_reliable;     // 0 = khong nen dua vao pos_hold luc nay (flow mat qua lau)
//} PosHold_t;
//
///* ================= API ================= */
//
//// Khoi tao toan bo struct (goi 1 lan luc boot, sau khi da tune PID gains ben trong pos_hold.c)
//void PosHold_Init(PosHold_t *ph);
//
//// Bat che do giu vi tri: reset KF ve 0 va lay vi tri hien tai (0,0 tuong doi) lam target
//void PosHold_Enable(PosHold_t *ph);
//
//// Tat che do giu vi tri (vd khi pilot dap stick roll/pitch de bay tay)
//void PosHold_Disable(PosHold_t *ph);
//
///*
// * Goi ham nay o tan so on dinh (khuyen nghi cung tan voi optical flow, ~50-100Hz,
// * hoac it nhat bang tan so goi MTF01_Update() tra ve 1).
// *
// * Tham so:
// *   mtf         : con tro toi struct MTF01_t da duoc MTF01_Update() cap nhat
// *   roll, pitch, yaw : rad, tu attitude estimator hien co cua ban
// *   gyro_p      : toc do goc quay quanh truc roll (rad/s)
// *   gyro_q      : toc do goc quay quanh truc pitch (rad/s)
// *   accel_x_body, accel_y_body : gia toc body frame da tru trong luc (m/s^2)
// *   dt          : giay
// *
// * Sau khi goi, doc ph->roll_sp va ph->pitch_sp de cong vao setpoint attitude.
// * Neu ph->is_reliable == 0, KHONG nen dung roll_sp/pitch_sp (flow bi mat qua lau).
// */
//void PosHold_Update(PosHold_t *ph, MTF01_t *mtf,
//                    float roll, float pitch, float yaw,
//                    float gyro_p, float gyro_q,
//                    float accel_x_body, float accel_y_body,
//                    float dt);
//
//#endif /* INC_POS_HOLD_H_ */
