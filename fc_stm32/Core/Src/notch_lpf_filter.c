/*
 * notch_lpf_fillter.c
 *
 *  Created on: Sep 23, 2026
 *      Author: lethanhtra
 */

#include "notch_lpf_filter.h"
#include <math.h>
#include "cmsis_gcc.h"

void Biquad_DesignNotch(Biquad_t *f, float fs, float f0, float Q) {
	float w0 = 2.0f * (float) M_PI * f0 / fs;
	float alpha = sinf(w0) / (2.0f * Q);
	float cosw0 = cosf(w0);
	float a0 = 1.0f + alpha;

	f->b0 = 1.0f / a0;
	f->b1 = (-2.0f * cosw0) / a0;
	f->b2 = 1.0f / a0;
	f->a1 = (-2.0f * cosw0) / a0;
	f->a2 = (1.0f - alpha) / a0;
	f->z1 = f->z2 = 0.0f;
}

void Biquad_DesignLPF(Biquad_t *f, float fs, float fc) {
	float w0 = 2.0f * (float) M_PI * fc / fs;
	float alpha = sinf(w0) / (2.0f * 0.7071f);   // Q=0.707 Butterworth
	float cosw0 = cosf(w0);
	float a0 = 1.0f + alpha;

	f->b1 = (1.0f - cosw0) / a0;
	f->b0 = f->b1 / 2.0f;
	f->b2 = f->b0;
	f->a1 = (-2.0f * cosw0) / a0;
	f->a2 = (1.0f - alpha) / a0;
	f->z1 = f->z2 = 0.0f;
}

static inline float Biquad_Update(Biquad_t *f, float in) {
	float out = f->b0 * in + f->z1;
	f->z1 = f->b1 * in + f->z2 - f->a1 * out;
	f->z2 = f->b2 * in - f->a2 * out;
	return out;
}

static void AxisFilter_Init(AxisFilter_t *af, float fs, float notch_f0,
		float notch_Q, float lpf_fc, uint8_t use_notch) {
	Biquad_DesignNotch(&af->notch, fs, notch_f0, notch_Q);
	Biquad_DesignLPF(&af->lpf, fs, lpf_fc);
	af->accum = 0.0f;
	af->count = 0;
	af->use_notch = use_notch;
}

static inline void AxisFilter_Push(AxisFilter_t *af, float raw) {
	float v = af->use_notch ? Biquad_Update(&af->notch, raw) : raw;
	v = Biquad_Update(&af->lpf, v);
	af->accum += v;
	af->count++;
}

static float AxisFilter_PopAverage(AxisFilter_t *af) {
	__disable_irq();
	float sum = af->accum;
	uint32_t cnt = af->count;
	af->accum = 0.0f;
	af->count = 0;
	__enable_irq();

	return (cnt > 0) ? (sum / (float) cnt) : 0.0f;
}

void IMU_FilterChain_Init(IMU_FilterChain_t *fc, float sample_rate_hz,
		float notch_freq_hz, float notch_Q, float gyro_lpf_hz,
		float accel_lpf_hz) {
	AxisFilter_Init(&fc->gx, sample_rate_hz, notch_freq_hz, notch_Q,
			gyro_lpf_hz, 1);
	AxisFilter_Init(&fc->gy, sample_rate_hz, notch_freq_hz, notch_Q,
			gyro_lpf_hz, 1);
	AxisFilter_Init(&fc->gz, sample_rate_hz, notch_freq_hz, notch_Q,
			gyro_lpf_hz, 1);

	AxisFilter_Init(&fc->ax, sample_rate_hz, notch_freq_hz, notch_Q,
			accel_lpf_hz, 0);
	AxisFilter_Init(&fc->ay, sample_rate_hz, notch_freq_hz, notch_Q,
			accel_lpf_hz, 0);
	AxisFilter_Init(&fc->az, sample_rate_hz, notch_freq_hz, notch_Q,
			accel_lpf_hz, 0);
}

void IMU_FilterChain_Push(IMU_FilterChain_t *fc, float gx, float gy, float gz,
		float ax, float ay, float az) {
	AxisFilter_Push(&fc->gx, gx);
	AxisFilter_Push(&fc->gy, gy);
	AxisFilter_Push(&fc->gz, gz);

	AxisFilter_Push(&fc->ax, ax);
	AxisFilter_Push(&fc->ay, ay);
	AxisFilter_Push(&fc->az, az);
}

void IMU_FilterChain_PopAverage(IMU_FilterChain_t *fc, float *gx, float *gy,
		float *gz, float *ax, float *ay, float *az) {
	*gx = AxisFilter_PopAverage(&fc->gx);
	*gy = AxisFilter_PopAverage(&fc->gy);
	*gz = AxisFilter_PopAverage(&fc->gz);

	*ax = AxisFilter_PopAverage(&fc->ax);
	*ay = AxisFilter_PopAverage(&fc->ay);
	*az = AxisFilter_PopAverage(&fc->az);
}

void IMU_FilterChain_UpdateNotchFreq(IMU_FilterChain_t *fc, float fs,
		float new_freq_hz, float Q) {
	Biquad_DesignNotch(&fc->gx.notch, fs, new_freq_hz, Q);
	Biquad_DesignNotch(&fc->gy.notch, fs, new_freq_hz, Q);
	Biquad_DesignNotch(&fc->gz.notch, fs, new_freq_hz, Q);
	/* z1/z2 giữ nguyên state cũ khi đổi hệ số — có gây transient nhỏ,
	 chấp nhận được vì throttle-based notch chỉ update vài trăm ms/lần */
}
