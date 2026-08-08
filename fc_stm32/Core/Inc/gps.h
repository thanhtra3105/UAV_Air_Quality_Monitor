#ifndef GPS_H
#define GPS_H

#include "main.h"

// Kích thước buffer (128 byte là dư sức chứa 1 dòng $GNGGA dài nhất)
#define GPS_DMA_BUF_SIZE 256

typedef struct {
    double latitude;   // Đổi từ float sang double
    char lat_dir;
    double longitude;  // Đổi từ float sang double
    char lon_dir;
    int fix_quality;
    int satellites;
    float altitude;    // Độ cao dùng float vẫn ổn vì phần nguyên nhỏ
    volatile uint8_t ready;
} GPS_Data_t;

// Các hàm giao tiếp
void GPS_Init_DMA(UART_HandleTypeDef *huart);
void GPS_Process(GPS_Data_t *myGPS);

// Hàm ngắt dành riêng cho chuẩn GPDMA + IDLE
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* GPS_H */
