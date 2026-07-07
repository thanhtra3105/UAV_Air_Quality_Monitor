#ifndef GPS_H
#define GPS_H

#include "main.h" // Chứa định nghĩa UART_HandleTypeDef của STM32 HAL

// Cấu trúc lưu trữ dữ liệu GPS
typedef struct {
    float latitude;
    char lat_dir;
    float longitude;
    char lon_dir;
    int fix_quality;
    int satellites;
    float altitude;
    uint8_t ready;
} GPS_Data_t;


// ================= CÁC HÀM XỬ LÝ =================
// Hàm khởi tạo GPS (truyền UART đang dùng vào đây)
void GPS_Init(UART_HandleTypeDef *huart);

// Hàm này để gọi bên trong ngắt HAL_UART_RxCpltCallback (Rất nhẹ)
void GPS_UART_RxCallback(UART_HandleTypeDef *huart);

// Hàm xử lý chuỗi NMEA, gọi trong vòng lặp while(1) của main.c
void GPS_Process(GPS_Data_t *myGPS);

#endif /* GPS_H */
