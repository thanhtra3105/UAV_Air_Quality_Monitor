#include "gps.h"
#include <string.h>
#include <stdlib.h>

// Các biến nội bộ chỉ dùng trong file gps.c
static UART_HandleTypeDef *gps_huart;
uint8_t rx_byte;
char gps_buffer[100];
static uint16_t rx_index = 0;
volatile uint8_t gps_ready = 0;

// Khởi tạo GPS và mồi ngắt nhận byte đầu tiên
void GPS_Init(UART_HandleTypeDef *huart) {
	gps_huart = huart;
	HAL_UART_Receive_IT(gps_huart, &rx_byte, 1);
}

// Hàm chuyển đổi định dạng NMEA (DDMM.MMMM) sang Decimal Degrees (Google Maps)
float NMEA_To_DecimalDegrees(float nmea_coord, char dir) {
	// 1. Lấy phần nguyên là Độ (Degrees)
	// Ví dụ: 1604.03705 / 100 = 16.0403705 -> Ép kiểu (int) = 16
	int degrees = (int) (nmea_coord / 100);

	// 2. Lấy phần Phút (Minutes)
	// Ví dụ: 1604.03705 - (16 * 100) = 04.03705
	float minutes = nmea_coord - (degrees * 100);

	// 3. Chuyển Phút thành Độ thập phân
	float decimal_degrees = degrees + (minutes / 60.0f);

	// 4. Xử lý dấu âm/dương (Nam và Tây là số âm)
	if (dir == 'S' || dir == 'W') {
		decimal_degrees = -decimal_degrees;
	}

	return decimal_degrees;
}

// Hàm nội bộ bóc tách chuỗi theo dấu phẩy
static void get_field(char *sentence, int field_num, char *output) {
	int current_field = 0;
	int i = 0, j = 0;

	while (sentence[i] != '\0' && sentence[i] != '*') {
		if (sentence[i] == ',') {
			current_field++;
			i++;
			continue;
		}
		if (current_field == field_num) {
			output[j++] = sentence[i];
		}
		if (current_field > field_num) {
			break;
		}
		i++;
	}
	output[j] = '\0';
}

// HÀM CHẠY TRONG NGẮT: Chỉ nhận byte và cắm cờ
void GPS_UART_RxCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == gps_huart->Instance) {
		if (rx_byte != '\n') {
			gps_buffer[rx_index++] = rx_byte;
			// Chống tràn buffer
			if (rx_index >= 100)
				rx_index = 0;
		} else {
			gps_buffer[rx_index] = '\0'; // Đóng chuỗi
			gps_ready = 1;               // Cắm cờ báo hiệu đã nhận xong 1 dòng
			rx_index = 0;                // Reset để nhận dòng mới
		}
		// Mồi lại ngắt để nhận byte tiếp theo
		HAL_UART_Receive_IT(gps_huart, &rx_byte, 1);
	}
}

// HÀM CHẠY TRONG MAIN (while 1): Xử lý chuỗi NMEA khi có cờ báo
void GPS_Process(GPS_Data_t *myGPS) {
	if (gps_ready == 1) {
		myGPS->ready = 1;
		// Chỉ xử lý nếu chuỗi là $GPGGA
		if (strncmp(gps_buffer, "$GNGGA", 6) == 0) {
			char tempBuf[20];
			float raw_lat = 0.0f;
			float raw_lon = 0.0f;

			// 1. LẤY VĨ ĐỘ (Trường số 2 và 3)
			get_field(gps_buffer, 2, tempBuf);
			if (tempBuf[0] != '\0')
				raw_lat = atof(tempBuf);

			get_field(gps_buffer, 3, tempBuf);
			if (tempBuf[0] != '\0')
				myGPS->lat_dir = tempBuf[0];

			// 2. LẤY KINH ĐỘ (Trường số 4 và 5)
			get_field(gps_buffer, 4, tempBuf);
			if (tempBuf[0] != '\0')
				raw_lon = atof(tempBuf);

			get_field(gps_buffer, 5, tempBuf);
			if (tempBuf[0] != '\0')
				myGPS->lon_dir = tempBuf[0];

			// 3. CHUYỂN ĐỔI VÀ GÁN VÀO ĐÚNG BIẾN
			// Chỉ tính toán khi có dữ liệu thật (GPS đã fix)
			if (raw_lat != 0.0f && raw_lon != 0.0f) {
				myGPS->latitude = NMEA_To_DecimalDegrees(raw_lat,
						myGPS->lat_dir);
				myGPS->longitude = NMEA_To_DecimalDegrees(raw_lon,
						myGPS->lon_dir);
			}
		}

		// Xử lý xong thì hạ cờ xuống
		gps_ready = 0;
		myGPS->ready = 0;
	}
}
