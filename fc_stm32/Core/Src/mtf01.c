/*
 * mtf01.c
 */

#include "mtf01.h"
#include <stdlib.h>
#include <string.h>

// Buffer nhận DMA theo mẻ
#define MTF01_RX_BUFFER_SIZE 64
uint8_t rx_mtf01_buffer[MTF01_RX_BUFFER_SIZE];

MICOLINK_MSG_t micolink_msg = { 0 };
volatile uint8_t mtf01_data_ready = 0;

#define FLOW_DEADBAND 2

// ---------------------------------------------------------
// CÁC HÀM PARSER CHUẨN TỪ TÀI LIỆU MICOAIR
// ---------------------------------------------------------
bool micolink_check_sum(MICOLINK_MSG_t *msg) {
	uint8_t length = msg->len + 6;
	uint8_t temp[MICOLINK_MAX_LEN];
	uint8_t checksum = 0;

	memcpy(temp, msg, length);

	for (uint8_t i = 0; i < length; i++) {
		checksum += temp[i];
	}

	if (checksum == msg->checksum)
		return true;
	else
		return false;
}

bool micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data) {
	switch (msg->status) {
	case 0:     // Header
		if (data == MICOLINK_MSG_HEAD) {
			msg->head = data;
			msg->status++;
		}
		break;
	case 1:     // Device ID
		msg->dev_id = data;
		msg->status++;
		break;
	case 2:     // System ID
		msg->sys_id = data;
		msg->status++;
		break;
	case 3:     // Message ID
		msg->msg_id = data;
		msg->status++;
		break;
	case 4:     // Sequence
		msg->seq = data;
		msg->status++;
		break;
	case 5:     // Payload Length
		msg->len = data;
		if (msg->len == 0)
			msg->status += 2;
		else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
			msg->status = 0;
		else
			msg->status++;
		break;
	case 6:     // Payload Data
		msg->payload[msg->payload_cnt++] = data;
		if (msg->payload_cnt == msg->len) {
			msg->payload_cnt = 0;
			msg->status++;
		}
		break;
	case 7:     // Checksum
		msg->checksum = data;
		msg->status = 0;
		if (micolink_check_sum(msg))
			return true;
		break;
	default:
		msg->status = 0;
		msg->payload_cnt = 0;
		break;
	}
	return false;
}

// ---------------------------------------------------------
// GIAO TIẾP VỚI STM32 HAL & XỬ LÝ DỮ LIỆU
// ---------------------------------------------------------

void MTF01_Init(UART_HandleTypeDef *huart) {
	memset(&micolink_msg, 0, sizeof(MICOLINK_MSG_t));
	// Khởi động DMA lắng nghe cho đến khi đường truyền rảnh (Idle)
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_mtf01_buffer, MTF01_RX_BUFFER_SIZE);
}

// Thay thế hoàn toàn hàm MTF01_Process cũ bằng hàm dưới đây
void MTF01_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// Quét từng byte trong mẻ dữ liệu DMA vừa đẩy vào
	for (uint16_t i = 0; i < Size; i++) {
		if (!mtf01_data_ready) {
			if (micolink_parse_char(&micolink_msg, rx_mtf01_buffer[i])) {
				mtf01_data_ready = 1;
			}
		}
	}
	// Khởi động lại DMA để đón mẻ dữ liệu tiếp theo
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_mtf01_buffer, MTF01_RX_BUFFER_SIZE);
}

// Hàm này gọi trong vòng lặp while(1) hoặc task của FreeRTOS
uint8_t MTF01_Update(MTF01_t *mtf) {
	if (mtf01_data_ready) {
		// Chỉ xử lý nếu đúng ID của cảm biến
		if (micolink_msg.msg_id == MICOLINK_MSG_ID_RANGE_SENSOR) {
			MICOLINK_PAYLOAD_RANGE_SENSOR_t payload;
			memcpy(&payload, micolink_msg.payload, micolink_msg.len);

			mtf->distance = payload.distance;
			mtf->dist_quality = payload.strength;
			mtf->flow_x = payload.flow_vel_x;
			mtf->flow_y = payload.flow_vel_y;
			mtf->flow_quality = payload.flow_quality;

			// Lọc nhiễu Deadband
			if (abs(mtf->flow_x) <= FLOW_DEADBAND)
				mtf->flow_x = 0;
			if (abs(mtf->flow_y) <= FLOW_DEADBAND)
				mtf->flow_y = 0;

			// Chỉ tính toán nếu Lidar đọc > 10mm và tín hiệu Flow ổn định
			if (mtf->flow_quality > 20 && mtf->distance >= 10) {
				float h_m = (float) mtf->distance * 0.001f;

				// Công thức hãng: V(cm/s) = Flow * H(m). Từ đó chia 100 để ra V(m/s)
				mtf->vx = (mtf->flow_x * h_m) / 100.0f;
				mtf->vy = (mtf->flow_y * h_m) / 100.0f;
			} else {
				mtf->vx = 0;
				mtf->vy = 0;
			}
		}

		mtf01_data_ready = 0; // Giải phóng cờ để đọc gói mới
		return 1;
	}
	return 0;
}

float MTF01_Median3(float x) {
	static float buf[3] = { 0 };
	static uint8_t index = 0;

	buf[index] = x;
	index = (index + 1) % 3;

	float a = buf[0];
	float b = buf[1];
	float c = buf[2];

	if (a > b) {
		float t = a;
		a = b;
		b = t;
	}

	if (b > c) {
		float t = b;
		b = c;
		c = t;
	}

	if (a > b) {
		float t = a;
		a = b;
		b = t;
	}

	return b;
}

void MTF01_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	// Kiểm tra xem lỗi có xuất phát từ UART của GPS không
	if (huart->Instance == USART2) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_UART_AbortReceive(huart);
		extern uint8_t uart_rx_mtf01;
		HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_mtf01_buffer,
		MTF01_RX_BUFFER_SIZE);
	}

}
