/**
 * @file fc_ahrs.c
 * @brief Attitude and Heading Reference System (AHRS) & Sensor Processing Implementation
 */

#include "fc_ahrs.h"
#include "dwt.h"
#include "serial.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

ICM20602_t imu;
DSP310_t dsp_sensor;
IST8310_Data_t ist8310;
Kalman4D_t kf_4d;
extern MTF01_t mtf_data;

#ifdef USE_QMC5883
QMC5883_t      mag;
#endif

static float heading_lpf = 0.0f;

static float DSP_CalibrationAltitude(uint8_t sample) {
	float h = 0;
	for (int i = 0; i < sample; i++) {
		while (!DSP310_Read(&dsp_sensor)) {
			HAL_Delay(1);
		}
		h += dsp_sensor.altitude;
		HAL_Delay(60);
	}
	return (float) h / sample;
}

void AHRS_Calibrate(VehicleState_t *veh) {
	if (veh == NULL)
		return;

	veh->ax_offset = -0.0146474605f;
	veh->ay_offset = -0.020620605f;
	veh->az_offset = 0.980093241f;
	veh->gx_offset = 0.379269421f;
	veh->gy_offset = -0.210610926f;
	veh->gz_offset = 0.219878733f;
}

void AHRS_Init(VehicleState_t *veh) {
	if (veh == NULL)
		return;

	ICM_CS_HIGH();
	HAL_Delay(10);

	DWT_Init();
	ICM20602_Init();

#ifdef USE_QMC5883
    QMC5883_Init(&hi2c2, &mag, 0x00, 0x01, 0x03, 0x01);
    mag.declination = 0;
#else
	IST8310_Init(&hi2c2, &ist8310);
#endif

	if (!DSP310_Init(&dsp_sensor, &hi2c1)) {
		Serial_printf(&huart1, "DSP Init Failed!\r\n");
	} else {
		Serial_printf(&huart1, "[OK] DSP Init Successful!\r\n");
	}

	veh->baro_alt_offset = DSP_CalibrationAltitude(10);

	while (!DSP310_Read(&dsp_sensor)) {
		HAL_Delay(1);
	}

	float h = dsp_sensor.altitude;
	Kalman4D_Init(&kf_4d, h - veh->baro_alt_offset);

	while (veh->range_alt_offset == 0) {
		MTF01_Update(&mtf_data);
		veh->range_alt_offset = mtf_data.distance / 10.0f;
	}
	AHRS_Calibrate(veh);
}

void AHRS_ReadIMU(VehicleState_t *veh) {
	if (veh == NULL)
		return;

	ICM20602_Read(&imu);
	veh->gx = (imu.gyro.x - veh->gx_offset);
	veh->gy = (imu.gyro.y - veh->gy_offset);
	veh->gz = -(imu.gyro.z - veh->gz_offset);

	veh->ax = imu.accel.x - veh->ax_offset;
	veh->ay = imu.accel.y - veh->ay_offset;
	veh->az = imu.accel.z - (veh->az_offset - 1.0f);
}

static float computeHeading(float bx, float by, float bz, float roll_deg,
		float pitch_deg) {
	float roll_rad = roll_deg * DEG_TO_RAD;
	float pitch_rad = pitch_deg * DEG_TO_RAD;
	float cr = cosf(roll_rad), sr = sinf(roll_rad);
	float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

	float mx = bx * cp + by * sr * sp + bz * cr * sp;
	float my = by * cr - bz * sr;

	float heading = atan2f(my, mx) * RAD_TO_DEG;
	if (heading > 180.0f)
		heading -= 360.0f;
	if (heading < -180.0f)
		heading += 360.0f;
	return heading;
}

float AHRS_ReadHeading(float roll_deg, float pitch_deg) {
#ifdef USE_QMC5883
    QMC5883_Read(&hi2c2, &mag);
    return computeHeading(mag.x, mag.y, mag.z, roll_deg, pitch_deg);
#else
	static float last_heading = 0.0f;
	if (IST8310_Read(&hi2c2, &ist8310) == IST8310_OK) {
		last_heading = computeHeading(ist8310.mag_y, -ist8310.mag_x,
				-ist8310.mag_z, roll_deg, pitch_deg);
	}
	return last_heading;
#endif
}

float range_cm = 0;
float range_vertical_cm = 0;

void AHRS_UpdateAttitude(VehicleState_t *veh, float dt) {
	if (veh == NULL)
		return;

	float roll_acc = atan2f(veh->ay,
			sqrtf(veh->ax * veh->ax + veh->az * veh->az)) * 57.2958f;
	float pitch_acc = atan2f(-veh->ax,
			sqrtf(veh->ay * veh->ay + veh->az * veh->az)) * 57.2958f;

	// Kalman 1D for Roll & Pitch
	Kalman1D_Compute(veh->roll, KalmanUncertaintyAngleRoll, veh->gx, roll_acc,
			dt);
	veh->roll = Kalman1DOutput[0];
	KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

	Kalman1D_Compute(veh->pitch, KalmanUncertaintyAnglePitch, veh->gy,
			pitch_acc, dt);
	veh->pitch = Kalman1DOutput[0];
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	// Yaw gyro integration + compass fusion
	veh->yaw += veh->gz * dt;

	float heading = AHRS_ReadHeading(veh->roll, veh->pitch);
	float mag_err = angle_diff_deg(heading, heading_lpf);

	heading_lpf += 0.1f * mag_err;
	if (heading_lpf > 180.0f)
		heading_lpf -= 360.0f;
	if (heading_lpf < -180.0f)
		heading_lpf += 360.0f;

	float err = angle_diff_deg(heading_lpf, veh->yaw);
	veh->yaw += 0.01f * err;

	if (veh->yaw > 180.0f)
		veh->yaw -= 360.0f;
	if (veh->yaw < -180.0f)
		veh->yaw += 360.0f;

	// Altitude Estimation - Inner loop: Vertical inertial acceleration
	float acc_z_inertial = -sinf(veh->pitch * DEG_TO_RAD) * veh->ax
			+ cosf(veh->pitch * DEG_TO_RAD) * sinf(veh->roll * DEG_TO_RAD)
					* veh->ay
			+ cosf(veh->pitch * DEG_TO_RAD) * cosf(veh->roll * DEG_TO_RAD)
					* veh->az;
	acc_z_inertial = (acc_z_inertial - 1.0f) * 981.0f;

	static bool acc_z_filt_init = false;
	const float LPF_ALPHA = 0.2f;
	if (!acc_z_filt_init) {
		veh->acc_z_filt = acc_z_inertial;
		acc_z_filt_init = true;
	} else {
		veh->acc_z_filt = LPF_ALPHA * acc_z_inertial
				+ (1.0f - LPF_ALPHA) * veh->acc_z_filt;
	}

	Kalman4D_Predict(&kf_4d, veh->acc_z_filt, dt);

	if (DSP310_Read(&dsp_sensor)) {

		veh->baro_alt = (dsp_sensor.altitude - veh->baro_alt_offset) * 100.0f;

		Kalman4D_UpdateBaro(&kf_4d, veh->baro_alt);

		/*MTF01*/
		float range_raw_cm = mtf_data.distance / 10.0f;
		range_cm = MTF01_Median3(range_raw_cm);

		uint8_t tilt_valid = (fabsf(veh->roll) < 25.0f)
				&& (fabsf(veh->pitch) < 25.0f);
		range_cm -= veh->range_alt_offset;
		if (range_cm < 0)
			range_cm = 0;

		uint8_t distance_valid = (range_cm < 1200.0f);
		uint8_t range_valid = 1;
		if (range_valid && tilt_valid && distance_valid) {
			float roll_rad = veh->roll * DEG_TO_RAD;
			float pitch_rad = veh->pitch * DEG_TO_RAD;

			range_vertical_cm = range_cm * cosf(roll_rad) * cosf(pitch_rad);
			Kalman4D_UpdateRange(&kf_4d, range_vertical_cm, veh->acc_z_filt);
		}

//
	}

	veh->est_alt = kf_4d.altitude;
	veh->est_vz = kf_4d.velocity;
}
