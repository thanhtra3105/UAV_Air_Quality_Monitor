/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *
 * ======Body frame of DRONE========
 *
 *           +X (Front)
 ↑
 │
 -Y (Left) ◄──┼──► +Y (Right)
 │
 ▼
 Rear
 *
 *===== AXis of Optical Flow====
 *          Front
 ↑
 │
 Vy <0 │
 │
 Left ◄──────┼──────► Right
 Vx <0  │        Vx >0
 │
 Vy >0 │
 ▼
 Rear
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf.h"
#include "icm20602.h"
#include "qmc5883.h"
#include "kalman.h"
#include "dwt.h"
#include "pid_controller.h"
#include "types.h"
#include <stdbool.h>
#include "uart_cmd.h"
#include "mtf01.h"
#include "tof.h"
#include "gps.h"
#include "pos_hold.h"
#include "ina219.h"
#include "dsp310.h"
#include "ist8310.h"
#include "kalman_position.h"
#include "kalman_gps.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define USE_QMC5883
#define USE_IST8310
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ICACHE_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RAD_TO_DEG 57.2958f
#define DEG_TO_RAD 0.017453f

uint8_t address[5] = { '0', '0', '0', '0', '1' };

/* THÊM ĐOẠN NÀY LÊN PHÍA TRÊN MAIN HOẶC VÀO USER CODE BEGIN */
#pragma pack(push, 1)
typedef struct {
	uint16_t trucX;
	uint16_t trucY;
	uint8_t trai;
	uint8_t phai;
	uint8_t len;
	uint8_t xuong;
	uint8_t batquathut;
	uint8_t nut1;
	uint8_t nut2;
	uint16_t chinhtocdoquat;
	uint8_t has_wp;
	int32_t wp_lat;
	int32_t wp_lon;
} ControlData;

// Struct gửi Telemetry về tay cầm (Giống hệt Arduino)
typedef struct {
	int32_t lat;
	int32_t lon;
	float x;
	float y;
	float alt;
	float target_x;
	float target_y;
	float battery;
} TelemetryData;

#pragma pack(pop)

ControlData rxData;
TelemetryData txData;
QMC5883_t mag;
ICM20602_t imu;
GPS_Data_t gps;
Kalman2D_t kf;
MTF01_t mtf_data;
KalmanAxis_t kf_x;
KalmanAxis_t kf_y;
KalmanAxis_t kf_x_gps;
KalmanAxis_t kf_y_gps;
DSP310_t dsp_sensor;
Kalman4D_t kf_4d;
IST8310_Data_t ist8310;
KalmanPos_t kf_pos_x;
KalmanPos_t kf_pos_y;
KalmanGPSAxis_t kf_x_gps_3d;
KalmanGPSAxis_t kf_y_gps_3d;

float dt;
float gx, gy, gz;
float ax, ay, az;
float ax_offset = 0.0;
float ay_offset = 0.0;
float az_offset = 0.0;
float gx_offset = 0.0;
float gy_offset = 0.0;
float gz_offset = 0.0;

// ===== PID Parameters =====
PIDController_t PID_Angle;
PIDController_t PID_Rate_Pitch;
PIDController_t PID_Rate_Roll;
PIDController_t PID_Rate_Yaw;
PIDController_t PID_Alt;
PIDController_t PID_Alt_Pos;   // outer: vị trí -> target velocity
PIDController_t PID_Alt_Vel;   // inner: velocity -> throttle offset

// ===== PID ANGLE =====
float kp_angle = 6.5, ki_angle = 0, kd_angle = 0;
// ===== PID RATE =====
float kp_r = 0.85, ki_r = 1.0, kd_r = 0.003;
float kp_p = 0.85, ki_p = 1.0, kd_p = 0.003;
float kp_y = 0.85, ki_y = 0.0, kd_y = 0.003;
// ===== PID ALTITUDE ====
//float kp_alt_pos = 4.0, ki_alt_pos = 0.0f, kd_alt_pos = 0.0f;		// Use MTF01 TOF
//float kp_alt_vel = 0.8, ki_alt_vel = 0.0f, kd_alt_vel = 0.003f;
float kp_alt_pos = 1.0, ki_alt_pos = 0.0f, kd_alt_pos = 0.0f;	// Use DSP310
float kp_alt_vel = 0.8, ki_alt_vel = 0.0f, kd_alt_vel = 0.003f;
// ===== PID POSITION HOLD ====
/*==== MTF01 ====*/
//float kp_xy_pos = 1.2f, ki_xy_pos = 0.0f, kd_xy_pos = 0.0f; //
//float kp_xy_vel = 6.0f, ki_xy_vel = 0.1f, kd_xy_vel = 0.05f; //
/*===== GPS ====*/
float kp_xy_pos = 0.8f, ki_xy_pos = 0.0f, kd_xy_pos = 0.0f; //
float kp_xy_vel = 2.0f, ki_xy_vel = 0.1f, kd_xy_vel = 0.1f; //

// ===== TARGET ROLL/PITCH MAX/MIN =======
const float MAX_TARGET_ROLL_PITCH = 8.0;

// ================= MOTOR =================
int throttle = 1000;
int throttle_hover = 1000;

// ================= ANGLE =================
float roll = 0;
float pitch = 0;
float yaw = 0;

// ================= TARGET ANGLE =================
float target_pitch = 0; // độ (angle mode)
float target_roll = 0;
float target_yaw = 0;
// ================= TARGET RATE =================
float target_rate_pitch = 0;              // deg/s
float target_rate_roll = 0;
float target_rate_yaw = 0;

// ============= ALTITUDE=======================
float kp_alt = 3.0, ki_alt = 0.0, kd_alt = 0.5;
float target_alt;
float pid_alt = 0;
float target_vz = 0.0f;
uint8_t mtf01_updated = 0;	// flag mtf updated
const float MAX_TARGET_VZ = 80.0f;
const float MAX_ALT_OUTPUT = 200.0f;

// ===== DSP310 ======
float alt_offset = 0;

// ================= OUTPUT PID =================
float pid_p, pid_r, pid_y;
float pid_alt_vel_out = 0.0f;

float yaw_filt = 0;

/*========SAFE CONNECTED========*/
uint32_t timeout_connected = 0;
uint32_t time_throttle = 0;

uint8_t pid_flag = 0;
int distance = 0;
float vx = 0;
float vy = 0;
float current_alt;
uint8_t alt_hold = 0;
uint8_t alt_init = 0;

extern char gps_buffer[100];
extern volatile uint8_t gps_ready;
extern uint8_t rx_byte;

float time_dt = 0;

char raw_gps_buffer[120];
uint16_t raw_index = 0;
volatile uint8_t line_ready = 0;

//==== EST POSITION=====
#define FLOW_ALPHA     0.2f
#define KP_FLOW        0.25f
#define KI_FLOW        0.02f
#define KP_POS         0.10f

float ax_earth = 0.0f;
float ay_earth = 0.0f;

float ax_bias = 0.0f;
float ay_bias = 0.0f;

// Vận tốc ước lượng (Earth Frame)
float est_vx = 0.0f;
float est_vy = 0.0f;

// Vị trí ước lượng (Earth Frame) - m
float est_x = 0.0f;
float est_y = 0.0f;

// Thêm vào vùng khai báo biến toàn cục
PIDController_t PID_Pos_X;
PIDController_t PID_Pos_Y;
PIDController_t PID_Vel_X;
PIDController_t PID_Vel_Y;

// Điểm neo (Target)
#define MAX_TARGET_VEL_XY 1.0f  // m/s, tùy kích thước/độ nhạy drone của bạn
float target_x = 0.0f;
float target_y = 0.0f;

float target_vx = 0.0f;
float target_vy = 0.0f;

uint8_t pos_hold_active = 0; // Cờ trạng thái

uint8_t status = 0;
float ist_heading = 0.0f;

// === BIẾN CHO GPS POSITION HOLD ===
double home_lat = 0.0f;
double home_lon = 0.0f;
float gps_x = 0.0f, gps_y = 0.0f;
float last_gps_x = 0.0f, last_gps_y = 0.0f;
float gps_vx = 0.0f, gps_vy = 0.0f;
uint8_t gps_hold_active = 0;
uint8_t gps_home_set = 0;

static float acc_z_filt = 0.0f;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART7) {
		UART_CMD_Process(&huart7);
//		GPS_UART_RxCallback(&huart1);
	}

	if (huart->Instance == USART2) {
		MTF01_Process(&huart2);
	}

//	GPS_UART_RxCallback(&huart1);
//	if (huart->Instance == USART1) {
//		// CHÌA KHÓA Ở ĐÂY: Chỉ cho phép lưu data khi mảng đang rảnh (line_ready == 0)
//		if (line_ready == 0) {
//			raw_gps_buffer[raw_index++] = rx_byte;
//
//			// Nếu gặp ký tự xuống dòng hoặc mảng sắp tràn
//			if (rx_byte == '\n' || raw_index >= 119) {
//				raw_gps_buffer[raw_index] = '\0'; // Chốt chuỗi
//				line_ready = 1;             // Khóa mảng lại, báo cho main xử lý
//			}
//		}
//
//		// Vẫn phải mồi lại ngắt (dù có lưu vào mảng hay không) để tránh ORE
//		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//	}
}

// Hàm Callback mặc định của HAL cho sự kiện Receive To IDLE
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// Đẩy sự kiện sang cho hàm xử lý của ta bên file gps.c
	GPS_UART_RxEventCallback(huart, Size);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		pid_flag = 1;
	}

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float data, float min, float max) {
	if (data <= min)
		return min;
	if (data >= max)
		return max;
	return data;
}
void writeMotor(uint8_t chanel, int16_t us) {
	__HAL_TIM_SET_COMPARE(&htim3, chanel, us);
}

void Failsafe_Task(void) {
	if (HAL_GetTick() - timeout_connected > 1000) {
		alt_hold = false;

		if (HAL_GetTick() - time_throttle > 400) {
			if (throttle > 1200) {
				throttle -= 5;
				time_throttle = HAL_GetTick();
			} else {
				if (mtf_data.distance < 200)	// <20cm
					throttle = 1000;
			}
		}
	}
}

float target_pitch_body = 0.0f, target_roll_body = 0.0f;
void RxController() {
	if (NRF24_Available()) {
		uint8_t len = NRF24_GetDynamicPayloadSize();

		if (len == sizeof(ControlData)) {
			NRF24_Read((uint8_t*) &rxData, len);
			throttle = rxData.chinhtocdoquat;
			target_roll = (float) map(rxData.trucX, 0, 100,
					(long) MAX_TARGET_ROLL_PITCH,
					(long) -MAX_TARGET_ROLL_PITCH);
			target_pitch = (float) map(rxData.trucY, 0, 100,
					(long) MAX_TARGET_ROLL_PITCH,
					(long) -MAX_TARGET_ROLL_PITCH);
			alt_hold = rxData.nut1;
			timeout_connected = HAL_GetTick();
		}
		txData.lat = (int32_t) (gps.latitude * 1e7);
		txData.lon = (int32_t) (gps.longitude * 1e7);
		txData.x = est_x;
		txData.y = est_y;
//		txData.alt = current_alt;
		txData.alt = yaw;
//		txData.target_x = target_roll_body;
//		txData.target_y = target_pitch_body;
		NRF24_WriteAckPayload(0, &txData, sizeof(TelemetryData));

	} else {
		Failsafe_Task();
	}
}

void readIMU() {
	ICM20602_Read(&imu);
	gx = (imu.gyro.x - gx_offset);
	gy = (imu.gyro.y - gy_offset);
	gz = -(imu.gyro.z - gz_offset);

	ax = imu.accel.x - ax_offset;
	ay = imu.accel.y - ay_offset;
	az = imu.accel.z - (az_offset - 1);
}

// Các thông số nhiễu đo lường (Measurement Noise) - Bạn cần tinh chỉnh thực tế
#define OPTICAL_FLOW_R 0.5f // Phương sai nhiễu đo vận tốc của Optical flow

void Estimate_Position_Kalman(float dt) {
	/*=============================
	 1. Rotation Body -> Earth (Giữ nguyên của bạn)
	 =============================*/
	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax + (cy * sp * sr - sy * cr) * ay
			+ (cy * sp * cr + sy * sr) * az;

	ay_earth = sy * cp * ax + (sy * sp * sr + cy * cr) * ay
			+ (sy * sp * cr - cy * sr) * az;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	/*=============================
	 2. Optical Flow -> Earth (Giữ nguyên của bạn)
	 =============================*/
	float body_vx = -mtf_data.vy; // Tiến/Lùi
	float body_vy = mtf_data.vx;  // Trái/Phải

	float flow_vx = body_vx * cy - body_vy * sy;
	float flow_vy = body_vx * sy + body_vy * cy;

	// (Tuỳ chọn: bạn có thể vẫn giữ Low Pass Filter cho flow_vx, flow_vy nếu nhiễu quá mạnh,
	// nhưng bản chất KF đã lo việc lọc nhiễu, bạn có thể truyền thẳng flow vào Update)

	/*=============================
	 3. Kalman PREDICT (Dựa vào IMU)
	 =============================*/
	// LƯU Ý: Nếu dùng KF 2-state, bạn nên trừ bias thủ công trước khi đưa vào Predict
	// nếu bạn vẫn muốn giữ cơ chế trừ bias cũ.
	float ax_input = ax_earth - ax_bias;
	float ay_input = ay_earth - ay_bias;

	KalmanAxis_Predict(&kf_x, ax_input, dt);
	KalmanAxis_Predict(&kf_y, ay_input, dt);

	/*=============================
	 4. Kalman UPDATE (Dựa vào Optical Flow)
	 =============================*/
	if (mtf_data.flow_quality > 50 && mtf_data.distance >= 100) {
		// Cập nhật KF bằng vận tốc đo được từ Flow
		KalmanAxis_UpdateVel(&kf_x, flow_vx, OPTICAL_FLOW_R);
		KalmanAxis_UpdateVel(&kf_y, flow_vy, OPTICAL_FLOW_R);

		// Cập nhật Bias theo kiểu cũ (Do KF 2 trạng thái không tự ước lượng bias)
		float err_vx = flow_vx - kf_x.vel;
		float err_vy = flow_vy - kf_y.vel;
		ax_bias += KI_FLOW * err_vx * dt;
		ay_bias += KI_FLOW * err_vy * dt;
	}

	/*=============================
	 5. Cập nhật biến State cho hàm Position Hold
	 =============================*/
	est_x = kf_x.pos;
	est_vx = kf_x.vel;

	est_y = kf_y.pos;
	est_vy = kf_y.vel;
}
static float computeHeading(float bx, float by, float bz, float roll_deg,
		float pitch_deg) {
	float roll_rad = roll_deg * DEG_TO_RAD;
	float pitch_rad = pitch_deg * DEG_TO_RAD;
	float cr = cosf(roll_rad), sr = sinf(roll_rad);
	float cp = cosf(pitch_rad), sp = sinf(pitch_rad);

	float mx = bx * cp + by * sr * sp + bz * cr * sp;
	float my = by * cr - bz * sr;

	float heading = atan2f(my, mx) * RAD_TO_DEG; /* atan2f trả về sẵn trong [-180,180] */
	if (heading > 180.0f)
		heading -= 360.0f;
	if (heading < -180.0f)
		heading += 360.0f;
	return heading;
}

float readHeading(float roll_deg, float pitch_deg) {
#ifdef USE_QMC5883
	QMC5883_Read(&hi2c2, &mag);
	return computeHeading(mag.x, mag.y, mag.z, roll_deg, pitch_deg);

#else   /* IST8310 */
	static float last_heading = 0.0f;

	if (IST8310_Read(&hi2c2, &ist8310) == IST8310_OK) {
		last_heading = computeHeading(ist8310.raw_y, -ist8310.raw_x,
				-ist8310.raw_z, roll_deg, pitch_deg);
	}
	return last_heading;

#endif
}

float readAltitude(float pressure_hPa)		// pressure unit hPa
{
	float h = 44330 * (1 - pow((pressure_hPa / 1013.25), (1 / 5.255)));
	return h;	// met
}

void calculateAngle(float dt) {		// 500Hz

	float roll_acc = atan2(ay, sqrt(ax * ax + az * az)) * 57.2958;
	float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958;
//  ================= KALMAN FILTER =================
	Kalman1D_Compute(roll, KalmanUncertaintyAngleRoll, gx, roll_acc, dt);
	roll = Kalman1DOutput[0];
	KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

	Kalman1D_Compute(pitch, KalmanUncertaintyAnglePitch, gy, pitch_acc, dt);
	pitch = Kalman1DOutput[0];
	KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

	//---------------- Yaw ----------------- //
	// 1. Predict bằng gyro
	yaw += gz * dt;
	if (yaw > 180.0f)
		yaw -= 360.0f;
	if (yaw < -180.0f)
		yaw += 360.0f;
	// 2. Heading từ la bàn
	float heading = readHeading(roll, pitch);
	static float heading_lpf = 0.0f;

	heading_lpf += 0.1f * (heading - heading_lpf);
	// 3. Sai số nhỏ nhất
	float err = heading_lpf - yaw;
	if (err > 180.0f)
		err -= 360.0f;

	if (err < -180.0f)
		err += 360.0f;
	// 4. Complementary Filter
	const float alpha = 0.01f;      // 0.01~0.05
	yaw += alpha * err;
	if (yaw > 180.0f)
		yaw -= 360.0f;
	if (yaw < -180.0f)
		yaw += 360.0f;

	/* ============ALTITUDE HOLD - INNER (Velocity, 500Hz) ===========*/
	float acc_z_inertial = -sin(pitch * DEG_TO_RAD) * ax
			+ cos(pitch * DEG_TO_RAD) * sin(roll * DEG_TO_RAD) * ay
			+ cos(pitch * DEG_TO_RAD) * cos(roll * DEG_TO_RAD) * az;
	acc_z_inertial = (acc_z_inertial - 1) * 981.0f;

//	static float acc_z_filt = 0.0f;
	static bool acc_z_filt_init = false;
	const float LPF_ALPHA = 0.2f;
	if (!acc_z_filt_init) {
		acc_z_filt = acc_z_inertial;
		acc_z_filt_init = true;
	} else {
		acc_z_filt = LPF_ALPHA * acc_z_inertial
				+ (1.0f - LPF_ALPHA) * acc_z_filt;
	}

//	Kalman2D_Predict(&kf, acc_z_filt, dt); // <-- dùng dt_rate (2ms), không phải dt_angle
//	static float tof_cm = 0;
//	if (mtf01_updated) {
//		tof_cm = mtf_data.distance * 0.1f;
//		tof_cm = tof_cm * cosf(roll * DEG_TO_RAD) * cosf(pitch * DEG_TO_RAD);
//		current_alt = tof_cm;
//		Kalman2D_Update(&kf, current_alt, 1.0f);
//		mtf01_updated = 0; // update done
//	}

	Kalman4D_Predict(&kf_4d, acc_z_filt, dt);
	static float dsp310_alt_cm = 0;

	if (DSP310_Read(&dsp_sensor)) {
		dsp310_alt_cm = (dsp_sensor.altitude - alt_offset) * 100;
		current_alt = dsp310_alt_cm;
		Kalman4D_Update(&kf_4d, current_alt, 5.0f);
	}
}

void Estimate_Position(float dt) {
	/*=============================
	 Rotation Body -> Earth
	 =============================*/
	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
//	float cy = 1.0f;
//	float sy = 0.0f;
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax + (cy * sp * sr - sy * cr) * ay
			+ (cy * sp * cr + sy * sr) * az;

	ay_earth = sy * cp * ax + (sy * sp * sr + cy * cr) * ay
			+ (sy * sp * cr - cy * sr) * az;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	/*=============================
	 Optical Flow -> Earth
	 =============================*/
	float body_vx = -mtf_data.vy; // Tiến/Lùi
	float body_vy = mtf_data.vx;  // Trái/Phải

	float flow_vx = body_vx * cy - body_vy * sy;
	float flow_vy = body_vx * sy + body_vy * cy;

	/*=============================
	 Low Pass Filter cho Flow
	 =============================*/
	static float flow_vx_lp = 0.0f;
	static float flow_vy_lp = 0.0f;

	flow_vx_lp += FLOW_ALPHA * (flow_vx - flow_vx_lp);
	flow_vy_lp += FLOW_ALPHA * (flow_vy - flow_vy_lp);

	/*=============================
	 Prediction (IMU)
	 =============================*/
	float ax_correct = ax_earth - ax_bias;
	float ay_correct = ay_earth - ay_bias;

	est_vx += ax_correct * dt;
	est_vy += ay_correct * dt;

	/*=============================
	 Correction (Optical Flow)
	 =============================*/
	// Cần > 100mm (10cm) để tránh Ground Effect
	if (mtf_data.flow_quality > 50 && mtf_data.distance >= 100) {
		float err_vx = flow_vx_lp - est_vx;
		float err_vy = flow_vy_lp - est_vy;

		ax_bias += KI_FLOW * err_vx * dt;
		ay_bias += KI_FLOW * err_vy * dt;

		est_vx += KP_FLOW * err_vx;
		est_vy += KP_FLOW * err_vy;
	}

	/*=============================
	 Integrate Position
	 =============================*/
	est_x += est_vx * dt;
	est_y += est_vy * dt;
}

void positionHold(float dt) {
	uint8_t pos_hold_sw = rxData.nut2;

	if (pos_hold_sw && alt_hold) {
		if (!pos_hold_active) {
			target_x = est_x;
			target_y = est_y;
			PID_Reset(&PID_Pos_X);
			PID_Reset(&PID_Pos_Y);
			PID_Reset(&PID_Vel_X);
			PID_Reset(&PID_Vel_Y);
			pos_hold_active = 1;
		}
		// ====== POSITION LOOP=====
		float err_x = target_x - est_x;
		float err_y = target_y - est_y;
		target_vx = PID_Calculate(&PID_Pos_X, err_x, dt);
		target_vy = PID_Calculate(&PID_Pos_Y, err_y, dt);

		target_vx = constrain(target_vx, -MAX_TARGET_VEL_XY, MAX_TARGET_VEL_XY);
		target_vy = constrain(target_vy, -MAX_TARGET_VEL_XY, MAX_TARGET_VEL_XY);
//		// ==== VELOCITY X,Y LOOP=====
//		float err_vx = target_vx - est_vx;
//		float err_vy = target_vy - est_vy;
//
//		// Góc nghiêng cần thiết trên hệ tọa độ Trái Đất
//		float out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt);
//		float out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt);
//
//		// VÒNG 3: ROTATION
//		float cy = cosf(yaw * DEG_TO_RAD);
//		float sy = sinf(yaw * DEG_TO_RAD);
//		float target_pitch_body = out_angle_earth_x * cy
//				+ out_angle_earth_y * sy;
//		float target_roll_body = -out_angle_earth_x * sy
//				+ out_angle_earth_y * cy;
//		// Giới hạn góc nghiêng tối đa khi giữ vị trí (VD: 15 độ)
//		target_pitch_body = constrain(target_pitch_body, -15.0f, 15.0f);
//		target_roll_body = constrain(target_roll_body, -15.0f, 15.0f);
//
//		target_pitch = -target_pitch_body; // Âm ngóc, dương chúi (Tùy cấu hình hàm cân bằng của bạn)
//		target_roll = target_roll_body;

		// ==== VELOCITY X,Y LOOP=====
		float err_vx = target_vx - est_vx;
		float err_vy = target_vy - est_vy;

		// 1. Góc nghiêng (hoặc Lực đẩy) cần thiết trên hệ tọa độ Trái Đất
		float out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt);
		float out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt);

		// 2. ĐẢO DẤU TẠI ĐÂY NẾU TRỤC BỊ NGƯỢC CHIỀU
		// Chuyển dấu trừ từ target_pitch lên đây!
		out_angle_earth_x = -out_angle_earth_x;

		// (Ghi chú: Nếu Roll của bạn cũng bị ngược thì thêm dấu trừ vào dòng dưới)
		// out_angle_earth_y = -out_angle_earth_y;

		// 3. VÒNG ROTATION (Xoay từ Earth -> Body)
		float cy = cosf(yaw * DEG_TO_RAD);
		float sy = sinf(yaw * DEG_TO_RAD);

		// Hàm xoay toán học nguyên bản, tuyệt đối không chèn thêm dấu ở đây
		target_pitch_body = out_angle_earth_x * cy + out_angle_earth_y * sy;
		target_roll_body = -out_angle_earth_x * sy + out_angle_earth_y * cy;

		// 4. Giới hạn góc nghiêng tối đa
		target_pitch_body = constrain(target_pitch_body, -15.0f, 15.0f);
		target_roll_body = constrain(target_roll_body, -15.0f, 15.0f);

		// 5. GÁN TRỰC TIẾP, KHÔNG CÒN DẤU TRỪ NÀO NỮA
		target_pitch = target_pitch_body;
		target_roll = target_roll_body;
	} else {
		// Tắt Pos Hold
		if (pos_hold_active) {
			pos_hold_active = 0;
			// Trả lại target_pitch và target_roll cho tay cầm (RC)
		}
	}
}

#define GPS_POS_R      2.5f
#define KI_GPS_BIAS    0.02f   // hệ số học bias — BẮT ĐẦU NHỎ, tune tăng dần
#define MAX_ACCEL_BIAS 1.0f    // m/s^2, chặn để tránh runaway khi GPS jump/nhiễu

void Estimate_Position_GPS_Kalman(float dt) {
	/*=============================
	 1. Xoay gia tốc từ Body -> Earth Frame
	 =============================*/
	float cy = cosf(yaw * DEG_TO_RAD);
	float sy = sinf(yaw * DEG_TO_RAD);
	float cp = cosf(pitch * DEG_TO_RAD);
	float sp = sinf(pitch * DEG_TO_RAD);
	float cr = cosf(roll * DEG_TO_RAD);
	float sr = sinf(roll * DEG_TO_RAD);

	ax_earth = cy * cp * ax + (cy * sp * sr - sy * cr) * ay
			+ (cy * sp * cr + sy * sr) * az;

	ay_earth = sy * cp * ax + (sy * sp * sr + cy * cr) * ay
			+ (sy * sp * cr - cy * sr) * az;

	ax_earth *= 9.81f;
	ay_earth *= 9.81f;

	/*=============================
	 2. Kalman PREDICT (Chạy 500Hz bằng IMU)
	 =============================*/
	KalmanGPSAxis_Predict(&kf_x_gps_3d, ax_earth, dt);
	KalmanGPSAxis_Predict(&kf_y_gps_3d, ay_earth, dt);

	/*=============================
	 3. Kalman UPDATE (Chạy ~10Hz bằng GPS)
	 =============================*/

	if (gps.fix_quality > 0 && gps.ready == 1) {
		if (!gps_home_set) {
			home_lat = gps.latitude;
			home_lon = gps.longitude;
			gps_home_set = 1;

			// Reset trạng thái
			kf_x_gps_3d.pos = 0.0f;
			kf_x_gps_3d.vel = 0.0f;
			kf_x_gps_3d.bias = 0.0f;
			kf_y_gps_3d.pos = 0.0f;
			kf_y_gps_3d.vel = 0.0f;
			kf_y_gps_3d.bias = 0.0f;
		} else {
			float lat_err = gps.latitude - home_lat;
			float lon_err = gps.longitude - home_lon;

			float new_gps_x = lat_err * 111320.0f;
			float new_gps_y = lon_err * 111320.0f * cosf(home_lat * DEG_TO_RAD);

			// Tính độ lệch xem có phải nhiễu (outlier) không
			float diff_x = new_gps_x - est_x;
			float diff_y = new_gps_y - est_y;

			if (sqrtf(diff_x * diff_x + diff_y * diff_y) < 5.0f) {
				gps_x = new_gps_x;
				gps_y = new_gps_y;

				KalmanGPSAxis_UpdatePos(&kf_x_gps_3d, gps_x, 1.0f);
				KalmanGPSAxis_UpdatePos(&kf_y_gps_3d, gps_y, 1.0f);
			} else {
				// Bỏ qua giá trị GPS bị lỗi
			}
		}
		gps.ready = 0;
	}

	// 4. Gán State cho hàm PID Position Hold
	est_x = kf_x_gps_3d.pos;
	est_vx = kf_x_gps_3d.vel;
	est_y = kf_y_gps_3d.pos;
	est_vy = kf_y_gps_3d.vel;
}

float err_x, err_y;
void GPS_PositionHold(float dt) {		// PID
	uint8_t gps_hold_sw = rxData.nut2;

	// Yêu cầu phải bật công tắc, có sóng GPS và đã chốt Home
	if (rxData.nut1 && gps_hold_sw && gps_home_set) {

		if (!pos_hold_active) {
			target_x = est_x; // Khóa vị trí hiện tại của Kalman
			target_y = est_y;
			PID_Reset(&PID_Pos_X);
			PID_Reset(&PID_Pos_Y);
			PID_Reset(&PID_Vel_X);
			PID_Reset(&PID_Vel_Y);
			pos_hold_active = 1;
		}

		// ==== VELOCITY X,Y LOOP=====
		float err_vx = target_vx - est_vx;
		float err_vy = target_vy - est_vy;

		// 1. Tính toán lực đẩy cần thiết trên hệ tọa độ Trái Đất (X=North, Y=East)
		float out_angle_earth_x = PID_Calculate(&PID_Vel_X, err_vx, dt);
		float out_angle_earth_y = PID_Calculate(&PID_Vel_Y, err_vy, dt);

		// 2. VÒNG ROTATION (Xoay từ Earth -> Body)
		float cy = cosf(yaw * DEG_TO_RAD);
		float sy = sinf(yaw * DEG_TO_RAD);

		// Phân tách lực Earth thành lực kéo trên hệ Body (Front và Right)
		// Tuyệt đối không tự ý thêm dấu trừ vào các công thức lượng giác này
		float force_body_x = out_angle_earth_x * cy + out_angle_earth_y * sy;
		float force_body_y = -out_angle_earth_x * sy + out_angle_earth_y * cy;

		// 3. Giới hạn góc nghiêng/lực tối đa
		force_body_x = constrain(force_body_x, -10.0f, 10.0f);
		force_body_y = constrain(force_body_y, -10.0f, 10.0f);

		// 4. MAPPING TRỰC TIẾP LÊN TRỤC CỦA DRONE (Gắn dấu)
		// Theo hệ thống của bạn: Tiến = Pitch âm, Phải = Roll dương
		target_pitch = force_body_x;
		target_roll = force_body_y;
		txData.target_x = target_roll;
		txData.target_y = target_pitch;
	} else {
		if (pos_hold_active) {
			pos_hold_active = 0;
			gps_home_set = 0;
			// Bỏ PosHold, phi công giành lại quyền điều khiển stick
			// Để an toàn, có thể reset gps_home_set = 0 khi tắt
		}
	}
}

void calculatePIDAngle(float dt_angle) {
	/* ============ALTITUDE HOLD - CASCADE===========*/
	if (alt_hold && !alt_init) {
		alt_init = true;
//		target_alt = kf.altitude;      // chốt độ cao hiện tại làm setpoint
		target_alt = kf_4d.altitude;
		PID_Reset(&PID_Alt_Pos);
		PID_Reset(&PID_Alt_Vel);
	}

	if (alt_hold) {
		float alt_err = target_alt - kf_4d.altitude;
		target_vz = PID_Calculate(&PID_Alt_Pos, alt_err, dt_angle);
		target_vz = constrain(target_vz, -MAX_TARGET_VZ, MAX_TARGET_VZ);
	} else {
		alt_init = false;
		target_vz = 0;
		PID_Reset(&PID_Alt_Pos);
	}

	//	========== OPTICAL FLOW MTF01 POS HOLD =============
//	positionHold(dt_angle);
	GPS_PositionHold(dt_angle);

	//  ================= PID ANGLE =================
	float pitch_err = target_pitch - pitch;
	target_rate_pitch = PID_Calculate(&PID_Angle, pitch_err, dt_angle);

	float roll_err = target_roll - roll;
	target_rate_roll = PID_Calculate(&PID_Angle, roll_err, dt_angle);

	float yaw_err = target_yaw - yaw;
	if (yaw_err > 180)
		yaw_err -= 360;
	if (yaw_err < -180)
		yaw_err += 360;
	target_rate_yaw = PID_Calculate(&PID_Angle, yaw_err, dt_angle);

	/* ======= CONSTRAINT =========*/
	target_rate_pitch = constrain(target_rate_pitch, -200, 200);
	target_rate_roll = constrain(target_rate_roll, -200, 200);
	target_rate_yaw = constrain(target_rate_yaw, -200, 200);
}

void calcualatePIDRate(float dt_rate) {
//  ================= PID RATE =================
	float pitch_rate_err = target_rate_pitch - gy;
	pid_p = PID_Calculate(&PID_Rate_Pitch, pitch_rate_err, dt_rate);

	float roll_rate_err = target_rate_roll - gx;
	pid_r = PID_Calculate(&PID_Rate_Roll, roll_rate_err, dt_rate);

	float yaw_rate_err = target_rate_yaw - gz;
	pid_y = PID_Calculate(&PID_Rate_Yaw, yaw_rate_err, dt_rate);

	if (alt_hold) {
//		float vz_err = target_vz - kf.velocity;	// tof
		float vz_err = target_vz - kf_4d.velocity;
		pid_alt_vel_out = PID_Calculate(&PID_Alt_Vel, vz_err, dt_rate);
		pid_alt = pid_alt_vel_out;
	} else {
		pid_alt = 0;
		PID_Reset(&PID_Alt_Vel);
	}

	pid_r = constrain(pid_r, -300, 300);
	pid_p = constrain(pid_p, -300, 300);
	pid_y = constrain(pid_y, -300, 300);
	pid_alt = constrain(pid_alt, -MAX_ALT_OUTPUT, MAX_ALT_OUTPUT);

}

int sp1, sp2, sp3, sp4;
void mixer() {
	int m1 = (int) (float) throttle - pid_p - pid_r - pid_y + pid_alt;
	int m2 = (int) (float) throttle + pid_p - pid_r + pid_y + pid_alt;
	int m3 = (int) (float) throttle + pid_p + pid_r - pid_y + pid_alt;
	int m4 = (int) (float) throttle - pid_p + pid_r + pid_y + pid_alt;

//	sp1 = m1;
//	sp2 = m2;
//	sp3 = m3;
//	sp4 = m4;

//	int m1 = (int) (float) throttle - pid_r;
//	int m2 = (int) (float) throttle - pid_r;
//	int m3 = (int) (float) throttle + pid_r;
//	int m4 = (int) (float) throttle + pid_r;

	if (throttle > 1040) {
		writeMotor(M1, m1);
		writeMotor(M2, m2);
		writeMotor(M3, m3);
		writeMotor(M4, m4);
	} else {
		writeMotor(M1, 1000);
		writeMotor(M2, 1000);
		writeMotor(M3, 1000);
		writeMotor(M4, 1000);
	}
}

static uint8_t angle_div = 0;

void PID_Task(float dt_pid) {
	if (++angle_div >= 2) {
		angle_div = 0;
		calculatePIDAngle(dt_pid * 2);      // 250 Hz
//		distance = readTOF(&hi2c1);
	}
	calcualatePIDRate(dt_pid);          // 500 Hz
}

void calibIMU() {
//	for (int i = 0; i < 500; i++) {
//		ICM20602_Read(&imu);
//		ax_offset += imu.accel.x;
//		ay_offset += imu.accel.y;
//		az_offset += imu.accel.z;
//
//		gx_offset += imu.gyro.x;
//		gy_offset += imu.gyro.y;
//		gz_offset += imu.gyro.z;
//	}
//	ax_offset = (ax_offset / 500);
//	ay_offset = (ay_offset / 500);
//	az_offset = (az_offset / 500);
//	gx_offset = (gx_offset / 500);
//	gy_offset = (gy_offset / 500);
//	gz_offset = (gz_offset / 500);

	// truong test
//	ax_offset = -0.0430698246;
//	ay_offset = -0.0115541993;
//	az_offset = 0.981339872;
//	gx_offset = 0.435978264;
//	gy_offset = -0.2842682;
//	gz_offset = 0.185610518;

	ax_offset = -0.0131186526;
	ay_offset = -0.0222158208;
	az_offset = 0.982454121;
	gx_offset = 0.564877927;
	gy_offset = -0.334876835;
	gz_offset = 0.182927459;
}

float DSP_CalibrationAltitude(uint8_t sample) {
	float h = 0;
	for (int i = 0; i < sample; i++) {
		DSP310_Read(&dsp_sensor);
		h += dsp_sensor.altitude;
		HAL_Delay(100);
	}
	return (float) h / sample;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_GPDMA1_Init();
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_ICACHE_Init();
	MX_UART7_Init();
	MX_TIM6_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_Base_Start_IT(&htim6);

	HAL_UART_Init(&huart7);
//	Serial_Init(&huart1, 115200);
	UART_Command_Init(&huart7);
//	MTF01_Init(&mtf01_handle, &huart2);
//	GPS_Init(&huart1);
	GPS_Init_DMA(&huart1);
	MTF01_Init(&huart2);

	HAL_I2C_Init(&hi2c1);
	HAL_I2C_Init(&hi2c2);
#ifdef USE_QMC5883
	QMC5883_Init(&hi2c2, &mag, 0x00, 0x01, 0x03, 0x01);
#else
	IST8310_Init(&hi2c2, &ist8310);
#endif
	HAL_SPI_Init(&hspi1);
	HAL_SPI_Init(&hspi2);
	ICM_CS_HIGH();
	NRF_CSN_HIGH();
	HAL_Delay(10);

	NRF24_Init();
	NRF24_OpenReadingPipe(0, address);
	NRF24_StartListening();

	HAL_Delay(500);
	DWT_Init();

	ICM20602_Init();
	mag.declination = 0;

	INA219_Init(&hi2c2);

	PID_Init(&PID_Angle, kp_angle, ki_angle, kd_angle, 0.2);
	PID_Init(&PID_Rate_Pitch, kp_p, ki_p, kd_p, 0.2);
	PID_SetIntegralLimits(&PID_Rate_Pitch, -60.0f, 60.0f);
	PID_Init(&PID_Rate_Roll, kp_r, ki_r, kd_r, 0.2);
	PID_SetIntegralLimits(&PID_Rate_Roll, -60.0f, 60.0f);
	PID_Init(&PID_Rate_Yaw, kp_y, ki_y, kd_y, 0.2);

	PID_Init(&PID_Alt, kp_alt, ki_alt, kd_alt, 0.2);
	PID_Init(&PID_Alt_Pos, kp_alt_pos, ki_alt_pos, kd_alt_pos, 0.2);
	PID_Init(&PID_Alt_Vel, kp_alt_vel, ki_alt_vel, kd_alt_vel, 0.2);
	PID_SetIntegralLimits(&PID_Alt_Vel, -40.0f, 40.0f);

	PID_Init(&PID_Pos_X, kp_xy_pos, ki_xy_pos, kd_xy_pos, 0.2);
	PID_Init(&PID_Pos_Y, kp_xy_pos, ki_xy_pos, kd_xy_pos, 0.2);

	PID_Init(&PID_Vel_X, kp_xy_vel, ki_xy_vel, kd_xy_vel, 0.2);
	PID_SetIntegralLimits(&PID_Vel_X, -6.0f, 6.0f); // 40% of 15deg
	PID_Init(&PID_Vel_Y, kp_xy_vel, ki_xy_vel, kd_xy_vel, 0.2);
	PID_SetIntegralLimits(&PID_Vel_Y, -6.0f, 6.0f);

	Kalman2D_Init(&kf, 16);
	KalmanAxis_Init(&kf_x);
	KalmanAxis_Init(&kf_y);
	KalmanAxis_Init(&kf_x_gps);
	KalmanAxis_Init(&kf_y_gps);

	KalmanPos_Init(&kf_pos_x, 0.004f, 0.05f, 1.5f);
	KalmanPos_Init(&kf_pos_y, 0.004f, 0.05f, 1.5f);
	KalmanGPSAxis_Init(&kf_x_gps_3d);
	KalmanGPSAxis_Init(&kf_y_gps_3d);
//	PosHold_Init(&posHold);

	if (!DSP310_Init(&dsp_sensor, &hi2c1)) {
		Serial_printf(&huart1, "DSP Init Failed!\r\n");
	} else
		Serial_printf(&huart1, "[OK] DSP Init Successful!\r\n");

	alt_offset = DSP_CalibrationAltitude(10);	// 10 sample

	DSP310_Read(&dsp_sensor);
	float h = dsp_sensor.altitude;
	Kalman4D_Init(&kf_4d, h - alt_offset);
	calibIMU();

	bool yaw_hold_init = false;

	Serial_printf(&huart1, "[WAITING] Waiting pull throttle to 0!\r\n");
	while (1) {
		if (NRF24_Available()) {
			uint8_t len = NRF24_GetDynamicPayloadSize();
			if (len == sizeof(ControlData)) {
				NRF24_Read((uint8_t*) &rxData, len);
				if (rxData.chinhtocdoquat == 1000)
					break;
			}
		}
	}
//	NRF24_WriteAckPayload(0, &txData, sizeof(TelemetryData));		// thong bao da ket noi thanh cong voi tx

	uint32_t ina219_timer = 0;
	uint32_t esp_timer = 0;
	Serial_printf(&huart1, "[OK] START\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint32_t start = DWT_GetMicros();
		uint32_t time_get = HAL_GetTick();
		dt = 0.002f;
		RxController();
		if (!yaw_hold_init && throttle > 1030) {
			target_yaw = yaw;
			// home_position = gpsRead();
			yaw_hold_init = true;
		}
		if (throttle < 1030) {
			yaw_hold_init = false;
			target_yaw = yaw;
		}
		if (throttle < 1300) {
			PID_Reset(&PID_Rate_Yaw);
			PID_Reset(&PID_Rate_Pitch);
			PID_Reset(&PID_Rate_Roll);
			PID_Reset(&PID_Angle);
		}
//		if (pid_flag) {		// timer interrupt
//			PID_task();
//			pid_flag = 0;
//		}
		mtf01_updated = MTF01_Update(&mtf_data);
		GPS_Process(&gps);
		readIMU();
		calculateAngle(dt);
		Estimate_Position_GPS_Kalman(dt);
//		Estimate_Position(dt);
//		Estimate_Position_Kalman(dt);
//		GPS_PositionHold(dt);
		PID_Task(dt);
		mixer();
		if (DWT_GetMicros() - ina219_timer > 500000) {
			float voltage = INA219_Read(&hi2c2);
//			Serial_printf(&huart1, "%f\r\n", voltage);
			if (voltage >= 0.0f) {
				txData.battery = voltage;
			}
			ina219_timer = DWT_GetMicros();
		}
//		if (DWT_GetMicros() - esp_timer > 300000) {
//			char gps_tx[100]; // Tăng kích thước mảng lên để chứa đủ 8 số
//			// Format: lat, lon, est_x, est_y, pitch, roll, home_lat, home_lon
//			sprintf(gps_tx, "%f,%f,%f,%f,%f,%f,%f,%f\n", gps.latitude,
//					gps.longitude, est_x, est_y, target_pitch, target_roll,
//					home_lat, home_lon);
//
//			HAL_UART_Transmit(&huart7, (uint8_t*) gps_tx, strlen(gps_tx), 100);
//			esp_timer = DWT_GetMicros();
//		}
		if (mtf_data.flow_quality >= 100)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		while ((DWT_GetMicros() - start) < 2000)
			;
//		time_dt = DWT_GetMicros() - start;
//		Serial_printf(&huart1, "dt=%.4f\r\n", time_dt);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_CSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.CSIState = RCC_CSI_ON;
	RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 125;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the programming delay
	 */
	__HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
 * @brief GPDMA1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPDMA1_Init(void) {

	/* USER CODE BEGIN GPDMA1_Init 0 */

	/* USER CODE END GPDMA1_Init 0 */

	/* Peripheral clock enable */
	__HAL_RCC_GPDMA1_CLK_ENABLE();

	/* GPDMA1 interrupt Init */
	HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

	/* USER CODE BEGIN GPDMA1_Init 1 */

	/* USER CODE END GPDMA1_Init 1 */
	/* USER CODE BEGIN GPDMA1_Init 2 */

	/* USER CODE END GPDMA1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x60808CD3;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00300F38;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void) {

	/* USER CODE BEGIN ICACHE_Init 0 */

	/* USER CODE END ICACHE_Init 0 */

	/* USER CODE BEGIN ICACHE_Init 1 */

	/* USER CODE END ICACHE_Init 1 */

	/** Enable instruction cache (default 2-ways set associative cache)
	 */
	if (HAL_ICACHE_Enable() != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ICACHE_Init 2 */

	/* USER CODE END ICACHE_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x7;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 0x7;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 249;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2499;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 249;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_TX_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, CE_Pin | SPI1_CS_Pin | GPIO_PIN_12,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | CSN_Pin | GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ICM_INT_GPIO_Port, ICM_INT_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : CE_Pin SPI1_CS_Pin PC12 */
	GPIO_InitStruct.Pin = CE_Pin | SPI1_CS_Pin | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 CSN_Pin PB8 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | CSN_Pin | GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PC8 PC9 PC10 PC11 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ICM_INT_Pin */
	GPIO_InitStruct.Pin = ICM_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ICM_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
