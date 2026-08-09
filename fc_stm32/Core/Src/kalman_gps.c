///*
// * kalman_gps.c
// *
// *  Created on: Aug 3, 2026
// *      Author: lethanhtra
// */
//
//
//#include "kalman_gps.h"
//
//void KalmanGPSAxis_Init(KalmanGPSAxis_t *kf) {
//    kf->pos = 0.0f;
//    kf->vel = 0.0f;
//    kf->bias = 0.0f;
//
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            kf->P[i][j] = 0.0f;
//        }
//    }
//
//    // Khởi tạo độ tin cậy ban đầu (P)
//    // Giá trị này thể hiện mức độ "không chắc chắn" lúc vừa bật máy
//    kf->P[0][0] = 1.0f;  // Vị trí (m^2)
//    kf->P[1][1] = 0.5f;  // Vận tốc ((m/s)^2)
//    kf->P[2][2] = 0.5f;  // Bias ((m/s^2)^2) - Để nhỏ vì bias ban đầu thường không lớn
//}
//
//void KalmanGPSAxis_Predict(KalmanGPSAxis_t *kf, float accel, float dt) {
//    // 1. Tính gia tốc thực tế (đã trừ Bias ước lượng được)
//    float acc_corrected = accel - kf->bias;
//
//    // 2. Cập nhật State: pos, vel.
//    // Lưu ý: bias được giả định là hằng số hoặc thay đổi rất chậm, nên bias không đổi trong bước Predict.
//    kf->pos += kf->vel * dt + 0.5f * acc_corrected * dt * dt;
//    kf->vel += acc_corrected * dt;
//
//    // 3. Xây dựng Ma trận Jacobian (F) của mô hình động học
//    float c0 = -0.5f * dt * dt; // Đạo hàm của pos theo bias
//    float c1 = -dt;             // Đạo hàm của vel theo bias
//    const float F[3][3] = {
//        {1.0f, dt,   c0},
//        {0.0f, 1.0f, c1},
//        {0.0f, 0.0f, 1.0f}
//    };
//
//    // 4. Tính P_pred = F * P * F^T
//    float FP[3][3];
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            FP[i][j] = 0.0f;
//            for (int k = 0; k < 3; k++) {
//                FP[i][j] += F[i][k] * kf->P[k][j];
//            }
//        }
//    }
//
//    float Ppred[3][3];
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            Ppred[i][j] = 0.0f;
//            for (int k = 0; k < 3; k++) {
//                Ppred[i][j] += FP[i][k] * F[j][k]; // F^T tương đương đổi chỉ số F[j][k]
//            }
//        }
//    }
//
//    // 5. Thêm Ma trận Nhiễu Quá trình (Q)
//    float var_acc = 0.01f;        // Phương sai nhiễu đo gia tốc (m/s^2)^2 - Tune thực tế
//    float var_bias = 0.0001f;    // Tốc độ trôi của bias (Random Walk) - Phải RẤT NHỎ
//
//    float dt2 = dt * dt;
//    float dt3 = dt2 * dt;
//    float dt4 = dt2 * dt2;
//
//    Ppred[0][0] += 0.25f * dt4 * var_acc;
//    Ppred[0][1] += 0.5f  * dt3 * var_acc;
//    Ppred[1][0] += 0.5f  * dt3 * var_acc;
//    Ppred[1][1] += dt2 * var_acc;
//    Ppred[2][2] += var_bias * dt;
//
//    // 6. Cập nhật lại P
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            kf->P[i][j] = Ppred[i][j];
//        }
//    }
//}
//
//void KalmanGPSAxis_UpdatePos(KalmanGPSAxis_t *kf, float pos_meas, float R) {
//    // Với GPS, ta chỉ đo được vị trí -> Ma trận quan sát H = [1, 0, 0]
//
//    // 1. Tính Innovation (y): Độ chênh lệch giữa GPS đo được và Vị trí dự đoán
//    float y = pos_meas - kf->pos;
//
//    // 2. Tính S = H * P * H^T + R
//    // Do H = [1, 0, 0], S triệt tiêu chỉ còn lại phần tử P[0][0] + R
//    float S = kf->P[0][0] + R;
//    if (S < 1e-6f) S = 1e-6f; // Chống chia cho 0
//
//    // 3. Tính Kalman Gain (K = P * H^T / S)
//    float K[3];
//    K[0] = kf->P[0][0] / S;
//    K[1] = kf->P[1][0] / S;
//    K[2] = kf->P[2][0] / S;
//
//    // 4. Hiệu chỉnh State (Bí quyết nằm ở K[2] sẽ tự động kéo bias về chuẩn)
//    kf->pos  += K[0] * y;
//    kf->vel  += K[1] * y;
//    kf->bias += K[2] * y;
//
//    // 5. Cập nhật ma trận hiệp phương sai P = (I - K * H) * P
//    // Tối ưu hóa ma trận: Do H = [1, 0, 0], dòng code dưới đây là dạng rút gọn toán học của ma trận 3x3
//    float Pnew[3][3];
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            Pnew[i][j] = kf->P[i][j] - K[i] * kf->P[0][j];
//        }
//    }
//
//    // 6. Lưu lại P
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            kf->P[i][j] = Pnew[i][j];
//        }
//    }
//}

#include "kalman_gps.h"

void KalmanGPS_Init(KalmanFilter1D_t *kf, float init_pos, float init_vel,
		float q_accel, float q_bias) {
	kf->pos = init_pos;
	kf->vel = init_vel;
	kf->bias = 0.0f;

	kf->Q_accel = q_accel;
	kf->Q_bias = q_bias;

	// Khởi tạo ma trận P (đường chéo = 1, còn lại = 0)
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			kf->P[i][j] = (i == j) ? 1.0f : 0.0f;
		}
	}
}
//
//void KalmanGPS_Predict(KalmanFilter1D_t *kf, float accel_earth, float dt) {
//	// 1. Dự đoán trạng thái (Kinematic equations)
//	// Vị trí = Vị trí cũ + Vận tốc * dt
////    kf->pos += kf->vel * dt;
////    // Vận tốc = Vận tốc cũ + Gia tốc thật (accel_earth - bias) * dt
////    kf->vel += (accel_earth - kf->bias) * dt;
//
//	float acc = accel_earth - kf->bias;
//
//	// Position prediction
//	kf->pos += kf->vel * dt + 0.5f * acc * dt * dt;
//
//	// Velocity prediction
//	kf->vel += acc * dt;
//
//	// 2. Cập nhật ma trận P (P = F*P*F^T + Q)
//	float P00 = kf->P[0][0], P01 = kf->P[0][1], P02 = kf->P[0][2];
//	float P10 = kf->P[1][0], P11 = kf->P[1][1], P12 = kf->P[1][2];
//	float P20 = kf->P[2][0], P21 = kf->P[2][1], P22 = kf->P[2][2];
//
//	kf->P[0][0] = P00 + dt * (P10 + P01) + dt * dt * P11;
//	kf->P[0][1] = P01 + dt * P11 - dt * P02 - dt * dt * P12;
//	kf->P[0][2] = P02 + dt * P12;
//
//	kf->P[1][0] = P10 + dt * P11 - dt * P20 - dt * dt * P21;
//	kf->P[1][1] = P11 - dt * (P21 + P12) + dt * dt * P22 + kf->Q_accel * dt;
//	kf->P[1][2] = P12 - dt * P22;
//
//	kf->P[2][0] = P20 - dt * P21;
//	kf->P[2][1] = P21 - dt * P22;
//	kf->P[2][2] = P22 + kf->Q_bias * dt;
//}

void KalmanGPS_Predict(KalmanFilter1D_t *kf, float accel_earth, float dt) {
	float acc = accel_earth - kf->bias;

	/* =========================
	 * 1. State prediction
	 * ========================= */

	kf->pos += kf->vel * dt + 0.5f * acc * dt * dt;

	kf->vel += acc * dt;

	/* =========================
	 * 2. State transition matrix
	 *
	 * x = [pos vel bias]
	 *
	 * pos' = pos + vel*dt - 0.5*bias*dt²
	 * vel' = vel - bias*dt
	 * bias'= bias
	 * ========================= */

	float F[3][3] = { { 1.0f, dt, -0.5f * dt * dt }, { 0.0f, 1.0f, -dt }, {
			0.0f, 0.0f, 1.0f } };

	/* =========================
	 * 3. P = F * P * F'
	 * ========================= */

	float FP[3][3] = { 0 };

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {

			for (int k = 0; k < 3; k++) {
				FP[i][j] += F[i][k] * kf->P[k][j];
			}
		}
	}

	float Pnew[3][3] = { 0 };

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {

			for (int k = 0; k < 3; k++) {
				Pnew[i][j] += FP[i][k] * F[j][k];
			}
		}
	}

	/* =========================
	 * 4. Process noise
	 * ========================= */

	float dt2 = dt * dt;
	float dt3 = dt2 * dt;
	float dt4 = dt2 * dt2;

	Pnew[0][0] += 0.25f * dt4 * kf->Q_accel;
	Pnew[0][1] += 0.5f * dt3 * kf->Q_accel;
	Pnew[1][0] += 0.5f * dt3 * kf->Q_accel;
	Pnew[1][1] += dt2 * kf->Q_accel;

	Pnew[2][2] += kf->Q_bias * dt;

	/* =========================
	 * 5. Copy back
	 * ========================= */

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			kf->P[i][j] = Pnew[i][j];
		}
	}
}

void KalmanGPS_Update(KalmanFilter1D_t *kf, float gps_pos, float gps_vel,
		float r_pos, float r_vel) {
	// 1. Tính độ lệch (Innovation = Measurement - Prediction)
	float y_pos = gps_pos - kf->pos;
	float y_vel = gps_vel - kf->vel;

	// 2. Tính ma trận S (S = H*P*H^T + R)
	float S00 = kf->P[0][0] + r_pos;
	float S01 = kf->P[0][1];
	float S10 = kf->P[1][0];
	float S11 = kf->P[1][1] + r_vel;

	// Nghịch đảo của ma trận S (2x2)
	float det = S00 * S11 - S01 * S10;
	if (det < 1e-6f)
		return; // Bảo vệ chia cho 0 hoặc S suy biến

	float invS00 = S11 / det;
	float invS01 = -S01 / det;
	float invS10 = -S10 / det;
	float invS11 = S00 / det;

	// 3. Tính Kalman Gain K (Ma trận 3x2 = P * H^T * S^-1)
	float K[3][2];
	for (int i = 0; i < 3; i++) {
		K[i][0] = kf->P[i][0] * invS00 + kf->P[i][1] * invS10;
		K[i][1] = kf->P[i][0] * invS01 + kf->P[i][1] * invS11;
	}

	// 4. Sửa sai cho trạng thái (X = X + K*Y)
	kf->pos += K[0][0] * y_pos + K[0][1] * y_vel;
	kf->vel += K[1][0] * y_pos + K[1][1] * y_vel;
	kf->bias += K[2][0] * y_pos + K[2][1] * y_vel; // Tự động nhận diện và sửa sai số phần cứng IMU

	// 5. Cập nhật lại ma trận P (P = (I - K*H)*P)
	float P_new[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			P_new[i][j] = kf->P[i][j]
					- (K[i][0] * kf->P[0][j] + K[i][1] * kf->P[1][j]);
		}
	}

	// Ghi đè P cũ bằng P mới
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			kf->P[i][j] = P_new[i][j];
		}
	}
}
