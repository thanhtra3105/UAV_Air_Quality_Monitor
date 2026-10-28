#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "types.h" // Sử dụng trực tiếp FlightMode_t từ types.h

#define MAX_WAYPOINTS       50
#define MAX_JSON_LINE_LEN   512
#define MISSION_BUFFER_SIZE 512 // Giữ lại macro này để main.c không bị lỗi biên dịch

// Cấu trúc lưu 1 Waypoint
typedef struct {
    uint16_t index;
    double lat;
    double lon;
    float alt;
} Waypoint_t;

// Cấu trúc quản lý Mission
typedef struct {
    Waypoint_t waypoints[MAX_WAYPOINTS];
    uint16_t count;
    bool is_receiving;
    bool is_ready;
} MissionData_t;

// Cấu trúc trạng thái lệnh điều khiển nhận từ Pi
typedef struct {
    bool is_armed;
    float target_alt;
    FlightMode_t mode; // Dùng trực tiếp enum từ types.h
    uint16_t next_wp_target;
    bool mission_started;
} FlightControlState_t;

// 1. Giữ nguyên hàm không tham số để tương thích với pos_hold.c (dòng 769)
void Mission_Init(void);

// 2. Hàm khởi tạo UART DMA IDLE (Gọi 1 lần ở đầu hàm main)
void Mission_UART_Init(UART_HandleTypeDef *huart);
// Hàm khôi phục DMA khi có lỗi UART mà KHÔNG làm mất Waypoint (Gọi trong ErrorCallback)
void Mission_UART_ErrorCallback(UART_HandleTypeDef *huart);
// 3. Các hàm xử lý dữ liệu nhận từ UART DMA (hỗ trợ cả 2 cách gọi)
void Mission_UartRxHandler(UART_HandleTypeDef *huart, uint16_t Size);
void Mission_ParseChunk(const uint8_t *data, uint16_t len);

// 4. Các hàm Getter & Setter
uint16_t Mission_GetCount(void);
Waypoint_t Mission_GetWaypoint(uint16_t index);
bool Mission_IsReady(void);
const FlightControlState_t* FlightControl_GetState(void);
void FlightControl_SetMode(FlightMode_t mode);
void Mission_Complete(void);
void Mission_SendTelemetry(void);

#endif /* MISSION_H */
