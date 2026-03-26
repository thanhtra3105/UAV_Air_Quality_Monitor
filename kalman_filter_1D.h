typedef struct {
    float v;      // velocity estimate (m/s)
    float P;      // uncertainty
} KalmanVel1D_t;

KalmanVel1D_t kf_vx = {0.0f, 1.0f};
KalmanVel1D_t kf_vy = {0.0f, 1.0f};

// Q: trust IMU bao nhiêu — tăng nếu IMU drift nhiều
// R: trust GPS bao nhiêu — M10 ~0.3-0.5 m/s accuracy
#define KF_Q  0.1f    // process noise (IMU)
#define KF_R  0.3f    // measurement noise (GPS velocity)

// Gọi mỗi 5ms (200Hz) — predict bằng IMU acceleration
void kalman_vel_predict(KalmanVel1D_t *kf, float accel, float dt) {
    kf->v += accel * dt;      // tích phân gia tốc
    kf->P += KF_Q * dt;       // uncertainty tăng theo thời gian
}

// Gọi mỗi 100ms (10Hz) — update bằng GPS velocity
void kalman_vel_update(KalmanVel1D_t *kf, float gps_vel) {
    float K = kf->P / (kf->P + KF_R);   // Kalman gain
    kf->v += K * (gps_vel - kf->v);     // correct
    kf->P *= (1.0f - K);                 // uncertainty giảm
}