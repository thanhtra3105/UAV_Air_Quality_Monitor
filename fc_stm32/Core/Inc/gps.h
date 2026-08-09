#ifndef GPS_H
#define GPS_H

#include "main.h"

// Kích thước buffer đủ chứa 1-2 frame UBX (UBX-NAV-PVT dài 100 bytes)
#define GPS_DMA_BUF_SIZE 256

// Struct lưu trữ dữ liệu UBX-NAV-PVT
typedef struct {
    uint32_t iTOW;      // ms

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;

    uint8_t fixType;
    uint8_t numSV;

    int32_t lon;        // 1e-7 deg
    int32_t lat;        // 1e-7 deg

    int32_t height;     // mm
    int32_t hMSL;       // mm

    uint32_t hAcc;      // mm	// hozitation accuracy
    uint32_t vAcc;      // mm	// vertical	accuracy

    int32_t velN;       // mm/s
    int32_t velE;       // mm/s
    int32_t velD;       // mm/s

    int32_t gSpeed;     // mm/s
    uint32_t sAcc;      // mm/s	// speed accuracy
    uint16_t pDOP;

    uint8_t valid;

    // Đã chuyển đổi sang hệ thập phân cho EKF/PID
    double latitude;
    double longitude;
    float altitude;

    volatile uint8_t ready; // Cờ báo hiệu dữ liệu mới
} GPS_Data_t;

// Các hàm giao tiếp
void GPS_Init_DMA(UART_HandleTypeDef *huart);
void GPS_Process(GPS_Data_t *myGPS);

// Hàm ngắt dành riêng cho chuẩn GPDMA + IDLE
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* GPS_H */
