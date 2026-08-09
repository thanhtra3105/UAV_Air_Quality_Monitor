#include "gps.h"
#include <string.h>

static UART_HandleTypeDef *gps_huart;

// Buffer nhận thô trực tiếp từ GPDMA
// Lưu ý: Đổi sang uint8_t vì UBX là dữ liệu nhị phân (chứa 0x00)
uint8_t rx_dma_buffer[GPS_DMA_BUF_SIZE];

// Buffer chính (chứa dữ liệu đã copy an toàn để xử lý)
uint8_t main_gps_buffer[GPS_DMA_BUF_SIZE];

volatile uint8_t gps_data_ready = 0;
volatile uint16_t gps_data_len = 0;

// Khởi tạo và "Mồi" DMA lần đầu tiên
void GPS_Init_DMA(UART_HandleTypeDef *huart) {
	gps_huart = huart;
	memset(rx_dma_buffer, 0, GPS_DMA_BUF_SIZE);

	// Bật DMA nhận dữ liệu kết hợp ngắt IDLE Line
	HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer, GPS_DMA_BUF_SIZE);
}

// Hàm này sẽ được gọi khi xảy ra ngắt IDLE (nhận xong 1 block dữ liệu)
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == gps_huart->Instance) {
		if (Size > 0 && Size <= GPS_DMA_BUF_SIZE) {
			// Copy nhanh dữ liệu từ DMA sang buffer chính để tránh bị đè
			memcpy(main_gps_buffer, rx_dma_buffer, Size);
			gps_data_len = Size;
			gps_data_ready = 1; // Cắm cờ báo cho hàm Main biết đã có chuỗi mới
		}

		// QUAN TRỌNG: Mồi lại DMA để nó tiếp tục nhận block tiếp theo
		HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer,
				GPS_DMA_BUF_SIZE);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	// Kiểm tra xem lỗi có xuất phát từ UART của GPS không
	if (huart->Instance == gps_huart->Instance) {
		// Hủy quá trình nhận hiện tại để dọn dẹp cờ lỗi
		HAL_UART_AbortReceive(huart);

		// Quan trọng: Mồi lại DMA để tiếp tục bắt tín hiệu
		HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer,
				GPS_DMA_BUF_SIZE);
	}
}

// =========================================================
// MÁY TRẠNG THÁI (STATE MACHINE) XỬ LÝ UBX
// =========================================================

#define UBX_NAV_PVT_CLASS   0x01
#define UBX_NAV_PVT_ID      0x07
#define UBX_NAV_PVT_LENGTH  92

static uint8_t ubx_payload[92];
static uint8_t ubx_state = 0;
static uint8_t ubx_class;
static uint8_t ubx_id;
static uint16_t ubx_length;
static uint16_t ubx_index;
static uint8_t ubx_ck_a;
static uint8_t ubx_ck_b;
static uint8_t ubx_rx_ck_a;
static uint8_t ubx_rx_ck_b;

static uint16_t UBX_U16(const uint8_t *p) {
	return ((uint16_t) p[0]) | ((uint16_t) p[1] << 8);
}

static uint32_t UBX_U32(const uint8_t *p) {
	return ((uint32_t) p[0]) | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16)
			| ((uint32_t) p[3] << 24);
}

static int32_t UBX_I32(const uint8_t *p) {
	return (int32_t) UBX_U32(p);
}

static void UBX_Checksum(uint8_t *data, uint16_t len, uint8_t *ck_a,
		uint8_t *ck_b) {
	uint8_t a = 0, b = 0;
	for (uint16_t i = 0; i < len; i++) {
		a = a + data[i];
		b = b + a;
	}
	*ck_a = a;
	*ck_b = b;
}

static void GPS_UBX_ParseByte(uint8_t c, GPS_Data_t *myGPS) {
	switch (ubx_state) {
	case 0:
		if (c == 0xB5)
			ubx_state = 1;
		break;
	case 1:
		if (c == 0x62)
			ubx_state = 2;
		else if (c == 0xB5)
			ubx_state = 1;
		else
			ubx_state = 0;
		break;
	case 2:
		ubx_class = c;
		if (ubx_class == UBX_NAV_PVT_CLASS)
			ubx_state = 3;
		else
			ubx_state = 0;
		break;
	case 3:
		ubx_id = c;
		if (ubx_id == UBX_NAV_PVT_ID)
			ubx_state = 4;
		else
			ubx_state = 0;
		break;
	case 4:
		ubx_length = c;
		ubx_state = 5;
		break;
	case 5:
		ubx_length |= ((uint16_t) c << 8);
		if (ubx_length == UBX_NAV_PVT_LENGTH) {
			ubx_index = 0;
			ubx_state = 6;
		} else
			ubx_state = 0;
		break;
	case 6:
		ubx_payload[ubx_index++] = c;
		if (ubx_index >= ubx_length)
			ubx_state = 7;
		break;
	case 7:
		ubx_rx_ck_a = c;
		ubx_state = 8;
		break;
	case 8:
		ubx_rx_ck_b = c;
		// Tính checksum
		{
			uint8_t data[96];
			data[0] = ubx_class;
			data[1] = ubx_id;
			data[2] = (uint8_t) (ubx_length & 0xFF);
			data[3] = (uint8_t) (ubx_length >> 8);
			for (uint16_t i = 0; i < ubx_length; i++)
				data[4 + i] = ubx_payload[i];

			UBX_Checksum(data, 4 + ubx_length, &ubx_ck_a, &ubx_ck_b);

			if ((ubx_ck_a == ubx_rx_ck_a) && (ubx_ck_b == ubx_rx_ck_b)) {
				// Checksum đúng, parse dữ liệu
				uint8_t *p = ubx_payload;
				myGPS->iTOW = UBX_U32(&p[0]);
				myGPS->year = UBX_U16(&p[4]);
				myGPS->month = p[6];
				myGPS->day = p[7];
				myGPS->hour = p[8];
				myGPS->min = p[9];
				myGPS->sec = p[10];
				myGPS->valid = p[11];
				myGPS->fixType = p[20];
				myGPS->numSV = p[23];
				myGPS->lon = UBX_I32(&p[24]);
				myGPS->lat = UBX_I32(&p[28]);
				myGPS->height = UBX_I32(&p[32]);
				myGPS->hMSL = UBX_I32(&p[36]);
				myGPS->hAcc = UBX_U32(&p[40]);
				myGPS->vAcc = UBX_U32(&p[44]);
				myGPS->velN = UBX_I32(&p[48]);
				myGPS->velE = UBX_I32(&p[52]);
				myGPS->velD = UBX_I32(&p[56]);
				myGPS->gSpeed = UBX_I32(&p[60]);
				myGPS->sAcc = UBX_U32(&p[68]);
				myGPS->pDOP = UBX_U16(&p[76]);

				// Chuyển đổi sang chuẩn Decimal Degrees cho Drone EKF
				myGPS->latitude = (double) myGPS->lat / 1e7;
				myGPS->longitude = (double) myGPS->lon / 1e7;
				myGPS->altitude = (float) myGPS->hMSL / 1000.0f; // Đổi mm ra m

				if (myGPS->fixType >= 3) {
					myGPS->ready = 1; // Tọa độ 3D Fix đã sẵn sàng
				}
			}
		}
		ubx_state = 0;
		break;
	default:
		ubx_state = 0;
		break;
	}
}

// Hàm xử lý chính gọi trong vòng lặp while(1)
void GPS_Process(GPS_Data_t *myGPS) {
	if (gps_data_ready == 1) {

		// Quét toàn bộ buffer nhận được và đưa từng byte vào State Machine
		for (uint16_t i = 0; i < gps_data_len; i++) {
			GPS_UBX_ParseByte(main_gps_buffer[i], myGPS);
		}

		gps_data_ready = 0; // Hạ cờ sau khi xử lý xong toàn bộ cụm buffer
	}
}

