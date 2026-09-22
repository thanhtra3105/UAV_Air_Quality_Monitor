/*
 * mtf01.h
 *
 * Tích hợp chuẩn Micolink Protocol từ tài liệu chính hãng
 */

#ifndef INC_MTF01_H_
#define INC_MTF01_H_

#include "stm32h5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// --- ĐỊNH NGHĨA GIAO THỨC MICOLINK ---
#define MICOLINK_MSG_HEAD            0xEF
#define MICOLINK_MAX_PAYLOAD_LEN     64
#define MICOLINK_MAX_LEN             (MICOLINK_MAX_PAYLOAD_LEN + 7)
#define MICOLINK_MSG_ID_RANGE_SENSOR 0x51

// Cấu trúc khung truyền chính
typedef struct {
    uint8_t head;
    uint8_t dev_id;
    uint8_t sys_id;
    uint8_t msg_id;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MICOLINK_MAX_PAYLOAD_LEN];
    uint8_t checksum;
    uint8_t status;
    uint8_t payload_cnt;
} MICOLINK_MSG_t;

// Cấu trúc Payload (Ép pack 1 byte để chống lệch bộ nhớ)
#pragma pack(1)
typedef struct {
    uint32_t time_ms;       // Thời gian hệ thống (ms)
    uint32_t distance;      // Khoảng cách (mm). Min=10, 0 là lỗi
    uint8_t  strength;      // Cường độ tín hiệu (Quality Lidar)
    uint8_t  precision;     // Độ chính xác
    uint8_t  tof_status;    // Trạng thái Lidar
    uint8_t  reserved1;
    int16_t  flow_vel_x;    // Vận tốc trục X
    int16_t  flow_vel_y;    // Vận tốc trục Y
    uint8_t  flow_quality;  // Độ tin cậy Flow
    uint8_t  flow_status;   // Trạng thái Flow
    uint16_t reserved2;
} MICOLINK_PAYLOAD_RANGE_SENSOR_t;
#pragma pack()

// --- CẤU TRÚC ĐẦU RA CHO FLIGHT CONTROLLER ---
typedef struct {
    int16_t flow_x;
    int16_t flow_y;
    uint8_t flow_quality;
    uint32_t distance;      // mm
    uint8_t dist_quality;
    float vx;               // Vận tốc X (m/s)
    float vy;               // Vận tốc Y (m/s)
} MTF01_t;

// Khai báo hàm
void MTF01_Init(UART_HandleTypeDef *huart);
void MTF01_Process(UART_HandleTypeDef *huart);
uint8_t MTF01_Update(MTF01_t *mtf);
void MTF01_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void MTF01_UART_ErrorCallback(UART_HandleTypeDef *huart);
float MTF01_Median3(float x);
#endif /* INC_MTF01_H_ */


