/*
 * notch_lpf_fillter.h
 *
 *  Created on: Sep 23, 2026
 *      Author: lethanhtra
 */

#ifndef INC_NOTCH_LPF_FILTER_H_
#define INC_NOTCH_LPF_FILTER_H_

#include <stdint.h>

typedef struct {
	float b0, b1, b2, a1, a2;
	float z1, z2;
} Biquad_t;

typedef struct {
	Biquad_t notch;
	Biquad_t lpf;
	float accum;
	uint32_t count;
	uint8_t use_notch; // 1 = áp notch trước LPF, 0 = chỉ LPF (dùng cho accel nếu chưa cần notch)
} AxisFilter_t;

typedef struct {
	AxisFilter_t gx, gy, gz;
	AxisFilter_t ax, ay, az;
} IMU_FilterChain_t;

/* Thiết kế filter — gọi 1 lần lúc init */
void Biquad_DesignNotch(Biquad_t *f, float fs, float f0, float Q);
void Biquad_DesignLPF(Biquad_t *f, float fs, float fc);

/* Khởi tạo toàn bộ filter chain cho 6 trục accel+gyro */
void IMU_FilterChain_Init(IMU_FilterChain_t *fc, float sample_rate_hz,
		float notch_freq_hz, float notch_Q, float gyro_lpf_hz,
		float accel_lpf_hz);

/* Nạp 1 raw sample (gọi trong DMA callback, tần số cao) */
void IMU_FilterChain_Push(IMU_FilterChain_t *fc, float gx, float gy, float gz,
		float ax, float ay, float az);

/* Lấy trung bình + reset accumulator (gọi trong main loop, tần số thấp hơn) */
void IMU_FilterChain_PopAverage(IMU_FilterChain_t *fc, float *gx, float *gy,
		float *gz, float *ax, float *ay, float *az);

/* Đổi lại tần số notch runtime (dùng cho throttle-based dynamic notch) */
void IMU_FilterChain_UpdateNotchFreq(IMU_FilterChain_t *fc, float fs,
		float new_freq_hz, float Q);

#endif /* INC_NOTCH_LPF_FILLTER_H_ */
