///*
// * pos_hold.c
// *
// * Created on: Jul 4, 2026
// * Author: lethanhtra
// */
//
//#include "pos_hold.h"
//#include <math.h>
//#include <string.h>
//
//#define DEG2RAD(x) ((x) * 0.0174532925f)
//#define GRAVITY    9.80665f
//
///* ================================================================
// *  PID GAINS - CAN TUNE THEO KHUNG DRONE CUA BAN
// *  Goi y ban dau (tham khao, KHONG phai gia tri cuoi cung):
// *    - Position PID: chi can P (I=0, D=0) vi output la van toc setpoint,
// *      ban than vong velocity da co tinh chat giam soc.
// *    - Velocity PID: P chiem chinh, mot chut D de giam overshoot,
// *      I nho de triet tieu drift do gio/lech tam.
// * ================================================================ */
//static const float POS_KP  = 0.6f;    // (m/s) / m       -> vi tri sai 1m -> muon van toc 0.6 m/s ve target
//static const float VEL_KP  = 0.12f;   // rad / (m/s)     -> van toc sai 1 m/s -> nghieng ~0.12 rad (~7 do)
//static const float VEL_KI  = 0.02f;
//static const float VEL_KD  = 0.01f;
//
//
///* ================================================================
// *                          POS HOLD API
// * ================================================================ */
//void PosHold_Init(PosHold_t *ph)
//{
//    memset(ph, 0, sizeof(PosHold_t));
//
//    KalmanAxis_Init(&ph->kf_x);
//    KalmanAxis_Init(&ph->kf_y);
//
//    float max_tilt_rad = DEG2RAD(POS_HOLD_MAX_TILT_DEG);
//
//    // Position PID: output la van toc setpoint (m/s), gioi han vd +-2 m/s
//    PID_Init(&ph->pid_pos_x, POS_KP, 0.0f, 0.0f, 0.2f);
//    PID_Init(&ph->pid_pos_y, POS_KP, 0.0f, 0.0f, 0.22);
//
//    // Velocity PID: output la goc nghieng setpoint (rad), gioi han theo MAX_TILT
//    PID_Init(&ph->pid_vel_x, VEL_KP, VEL_KI, VEL_KD, 0.2f);
//    PID_Init(&ph->pid_vel_y, VEL_KP, VEL_KI, VEL_KD, 0.2f);
//
//    ph->state = POS_HOLD_DISABLED;
//    ph->is_reliable = 0;
//}
//
//void PosHold_Enable(PosHold_t *ph)
//{
//    // Reset KF ve 0 -> vi tri hien tai duoc coi la goc toa do (0,0)
//    KalmanAxis_Init(&ph->kf_x);
//    KalmanAxis_Init(&ph->kf_y);
//
//    PID_Reset(&ph->pid_pos_x);
//    PID_Reset(&ph->pid_pos_y);
//    PID_Reset(&ph->pid_vel_x);
//    PID_Reset(&ph->pid_vel_y);
//
//    ph->target_x = 0.0f;
//    ph->target_y = 0.0f;
//
//    ph->roll_sp = 0.0f;
//    ph->pitch_sp = 0.0f;
//
//    ph->low_quality_ms = 0;
//    ph->is_reliable = 1;
//
//    ph->state = POS_HOLD_ENABLED;
//}
//
//void PosHold_Disable(PosHold_t *ph)
//{
//    ph->state = POS_HOLD_DISABLED;
//    ph->roll_sp = 0.0f;
//    ph->pitch_sp = 0.0f;
//
//    PID_Reset(&ph->pid_pos_x);
//    PID_Reset(&ph->pid_pos_y);
//    PID_Reset(&ph->pid_vel_x);
//    PID_Reset(&ph->pid_vel_y);
//}
//
//void PosHold_Update(PosHold_t *ph, MTF01_t *mtf,
//                    float roll, float pitch, float yaw,
//                    float gyro_p, float gyro_q,
//                    float accel_x_body, float accel_y_body,
//                    float dt)
//{
//    if (dt <= 0.0f) dt = 0.001f;
//
//    float h = mtf->distance * 0.001f;   // mm -> m, giong cach mtf01.c dang tinh
//
//    uint8_t flow_valid = (mtf->quality > MTF01_QUALITY_MIN)
//                       && (h > MTF01_HEIGHT_MIN)
//                       && (h < MTF01_HEIGHT_MAX);
//
//    /* -------------------------------------------------------------
//     * 1) Bu anh huong xoay (rotation-induced flow)
//     *    LUU Y VE TRUC: chieu gan cam bien anh huong truc nao khop
//     *    voi truc goc quay nao (roll/pitch) phai duoc KIEM TRA THUC TE
//     *    (VD: dat drone tren gia, giu vi tri co dinh, xoay tay quanh
//     *    truc roll/pitch, xem vx/vy nhay ra sao) roi chinh lai neu can.
//     *    Cong thuc: v_true = v_do - gyro_rate * h
//     * ----------------------------------------------------------- */
//    float vx_body = mtf->vx - gyro_q * h;   // gia dinh: flow_x <-> pitch rate (gyro_q)
//    float vy_body = mtf->vy - gyro_p * h;   // gia dinh: flow_y <-> roll  rate (gyro_p)
//
//    /* -------------------------------------------------------------
//     * 2) Xoay tu body frame sang world frame (theo yaw)
//     *    Xap xi goc nho: bo qua anh huong cua roll/pitch len phep chieu
//     *    (dung cho hover, nghieng nho <15 do). Neu bay nghieng nhieu,
//     *    can ma tran xoay 3D day du.
//     * ----------------------------------------------------------- */
//    float cy = cosf(yaw), sy = sinf(yaw);
//    float v_world_x = vx_body * cy - vy_body * sy;   // "North" tuong doi
//    float v_world_y = vx_body * sy + vy_body * cy;   // "East"  tuong doi
//
//    float ax_world = accel_x_body * cy - accel_y_body * sy;
//    float ay_world = accel_x_body * sy + accel_y_body * cy;
//
//    /* -------------------------------------------------------------
//     * 3) Kalman filter predict (luon chay, du flow co hop le hay khong)
//     * ----------------------------------------------------------- */
//    KalmanAxis_Predict(&ph->kf_x, ax_world, dt);
//    KalmanAxis_Predict(&ph->kf_y, ay_world, dt);
//
//    /* -------------------------------------------------------------
//     * 4) Kalman filter update (chi khi flow dang tin cay)
//     *    R giam khi quality cao -> tin tuong phep do hon
//     * ----------------------------------------------------------- */
//    if (flow_valid) {
//        float quality_norm = (float)mtf->quality / 255.0f;
//        if (quality_norm < 0.1f) quality_norm = 0.1f;
//        float R = KF_R_BASE / quality_norm;
//
//        KalmanAxis_UpdateVel(&ph->kf_x, v_world_x, R);
//        KalmanAxis_UpdateVel(&ph->kf_y, v_world_y, R);
//
//        ph->low_quality_ms = 0;
//        ph->is_reliable = 1;
//    } else {
//        ph->low_quality_ms += (uint32_t)(dt * 1000.0f);
//        if (ph->low_quality_ms > POS_HOLD_LOWQ_TIMEOUT_MS) {
//            ph->is_reliable = 0;   // mat flow qua lau -> khong nen dua vao pos_hold nua
//        }
//    }
//
//    /* -------------------------------------------------------------
//     * 5) Vong dieu khien cascade (chi chay khi da bat va con tin cay)
//     * ----------------------------------------------------------- */
//    if (ph->state == POS_HOLD_ENABLED && ph->is_reliable) {
//
//        float pos_err_x = ph->target_x - ph->kf_x.pos;
//        float pos_err_y = ph->target_y - ph->kf_y.pos;
//
//        float vel_sp_x = PID_Calculate(&ph->pid_pos_x, pos_err_x, dt);
//        float vel_sp_y = PID_Calculate(&ph->pid_pos_y, pos_err_y, dt);
//
//        float vel_err_x = vel_sp_x - ph->kf_x.vel;
//        float vel_err_y = vel_sp_y - ph->kf_y.vel;
//
//        // Output truc tiep la goc nghieng world-frame (rad)
//        float tilt_world_x = PID_Calculate(&ph->pid_vel_x, vel_err_x, dt);
//        float tilt_world_y = PID_Calculate(&ph->pid_vel_y, vel_err_y, dt);
//
//        /* -------------------------------------------------------------
//         * 6) Xoay nguoc lai (theo -yaw) tu world frame ve body frame
//         *    de ra lenh roll/pitch setpoint dung theo huong mui drone
//         *
//         *    QUY UOC DAU: pitch am (nose-down) thuong tao ra gia toc
//         *    tien ve phia truoc; roll duong thuong nghieng sang phai.
//         *    Kiem tra lai dau (+/-) cho khop voi attitude controller
//         *    hien co cua ban truoc khi bay thu (bay tay giu do cao,
//         *    quan sat chieu nghieng khi bat pos_hold).
//         * ----------------------------------------------------------- */
//        float tilt_body_x =  tilt_world_x * cy + tilt_world_y * sy;
//        float tilt_body_y = -tilt_world_x * sy + tilt_world_y * cy;
//
//        ph->pitch_sp = -tilt_body_x;   // TODO: verify dau (sign) thuc te
//        ph->roll_sp  =  tilt_body_y;   // TODO: verify dau (sign) thuc te
//
//    } else {
//        ph->roll_sp = 0.0f;
//        ph->pitch_sp = 0.0f;
//    }
//
//    (void)roll;
//    (void)pitch;
//}
