//#include "gps.h"
//#include <string.h>
//#include <stdlib.h>
//
//// Các biến nội bộ chỉ dùng trong file gps.c
//static UART_HandleTypeDef *gps_huart;
//uint8_t rx_byte;
//char gps_buffer[100];
//static uint16_t rx_index = 0;
//volatile uint8_t gps_ready = 0;
//
//// Khởi tạo GPS và mồi ngắt nhận byte đầu tiên
//void GPS_Init(UART_HandleTypeDef *huart) {
//	gps_huart = huart;
//	HAL_UART_Receive_IT(gps_huart, &rx_byte, 1);
//}
//
//// Hàm chuyển đổi định dạng NMEA (DDMM.MMMM) sang Decimal Degrees (Google Maps)
//float NMEA_To_DecimalDegrees(float nmea_coord, char dir) {
//	// 1. Lấy phần nguyên là Độ (Degrees)
//	// Ví dụ: 1604.03705 / 100 = 16.0403705 -> Ép kiểu (int) = 16
//	int degrees = (int) (nmea_coord / 100);
//
//	// 2. Lấy phần Phút (Minutes)
//	// Ví dụ: 1604.03705 - (16 * 100) = 04.03705
//	float minutes = nmea_coord - (degrees * 100);
//
//	// 3. Chuyển Phút thành Độ thập phân
//	float decimal_degrees = degrees + (minutes / 60.0f);
//
//	// 4. Xử lý dấu âm/dương (Nam và Tây là số âm)
//	if (dir == 'S' || dir == 'W') {
//		decimal_degrees = -decimal_degrees;
//	}
//
//	return decimal_degrees;
//}
//
//// Hàm nội bộ bóc tách chuỗi theo dấu phẩy
//static void get_field(char *sentence, int field_num, char *output) {
//	int current_field = 0;
//	int i = 0, j = 0;
//
//	while (sentence[i] != '\0' && sentence[i] != '*') {
//		if (sentence[i] == ',') {
//			current_field++;
//			i++;
//			continue;
//		}
//		if (current_field == field_num) {
//			output[j++] = sentence[i];
//		}
//		if (current_field > field_num) {
//			break;
//		}
//		i++;
//	}
//	output[j] = '\0';
//}
//
//void GPS_UART_RxCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == gps_huart->Instance) {
//        // Bỏ qua ký tự \r, chỉ xử lý khi gặp \n
//        if (rx_byte != '\n') {
//            if (rx_byte != '\r') {
//                gps_buffer[rx_index++] = rx_byte;
//                // Chống tràn buffer
//                if (rx_index >= 100) rx_index = 0;
//            }
//        } else {
//            gps_buffer[rx_index] = '\0'; // Đóng chuỗi
//            gps_ready = 1;               // Cắm cờ báo hiệu đã nhận xong 1 dòng
//            rx_index = 0;                // Reset để nhận dòng mới
//        }
//        // Mồi lại ngắt để nhận byte tiếp theo
//        HAL_UART_Receive_IT(gps_huart, &rx_byte, 1);
//    }
//}
//
//void GPS_Process(GPS_Data_t *myGPS) {
//    if (gps_ready == 1) {
//        // Kiểm tra xem là GPGGA hay GNGGA
//        if (strncmp(gps_buffer, "$GPGGA", 6) == 0 || strncmp(gps_buffer, "$GNGGA", 6) == 0) {
//            char tempBuf[20];
//            float raw_lat = 0.0f;
//            float raw_lon = 0.0f;
//
//            // 1. LẤY VĨ ĐỘ (Trường số 2 và 3)
//            get_field(gps_buffer, 2, tempBuf);
//            if (tempBuf[0] != '\0') raw_lat = atof(tempBuf);
//
//            get_field(gps_buffer, 3, tempBuf);
//            if (tempBuf[0] != '\0') myGPS->lat_dir = tempBuf[0];
//
//            // 2. LẤY KINH ĐỘ (Trường số 4 và 5)
//            get_field(gps_buffer, 4, tempBuf);
//            if (tempBuf[0] != '\0') raw_lon = atof(tempBuf);
//
//            get_field(gps_buffer, 5, tempBuf);
//            if (tempBuf[0] != '\0') myGPS->lon_dir = tempBuf[0];
//
//            // 3. LẤY CHẤT LƯỢNG TÍN HIỆU VÀ SỐ VỆ TINH (Trường 6 và 7)
//            get_field(gps_buffer, 6, tempBuf);
//            if (tempBuf[0] != '\0') myGPS->fix_quality = atoi(tempBuf);
//
//            get_field(gps_buffer, 7, tempBuf);
//            if (tempBuf[0] != '\0') myGPS->satellites = atoi(tempBuf);
//
//            // 4. LẤY ĐỘ CAO (Trường số 9)
//            get_field(gps_buffer, 9, tempBuf);
//            if (tempBuf[0] != '\0') myGPS->altitude = atof(tempBuf);
//
//            // 5. CHUYỂN ĐỔI VÀ GÁN VÀO ĐÚNG BIẾN
//            if (raw_lat != 0.0f && raw_lon != 0.0f && myGPS->fix_quality > 0) {
//                myGPS->latitude = NMEA_To_DecimalDegrees(raw_lat, myGPS->lat_dir);
//                myGPS->longitude = NMEA_To_DecimalDegrees(raw_lon, myGPS->lon_dir);
//
//                // Cắm cờ ready lên 1 để báo cho main() biết có tọa độ hợp lệ mới
//                myGPS->ready = 1;
//            }
//        }
//
//        // Hạ cờ ngắt của UART
//        gps_ready = 0;
//    }
//}


#include "gps.h"
#include <string.h>
#include <stdlib.h>

static UART_HandleTypeDef *gps_huart;

// Buffer nhận thô trực tiếp từ GPDMA
uint8_t rx_dma_buffer[GPS_DMA_BUF_SIZE];

// Buffer chính (chứa dữ liệu đã copy an toàn để xử lý)
char main_gps_buffer[GPS_DMA_BUF_SIZE];

volatile uint8_t gps_data_ready = 0;
volatile uint16_t gps_data_len = 0;

// Khởi tạo và "Mồi" DMA lần đầu tiên
void GPS_Init_DMA(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    memset(rx_dma_buffer, 0, GPS_DMA_BUF_SIZE);

    // Bật DMA nhận dữ liệu kết hợp ngắt IDLE Line
    HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer, GPS_DMA_BUF_SIZE);
}

// Hàm này sẽ được gọi khi xảy ra ngắt IDLE (nhận xong 1 câu lệnh)
void GPS_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == gps_huart->Instance) {
        if (Size > 0 && Size < GPS_DMA_BUF_SIZE) {
            // Copy nhanh dữ liệu từ DMA sang buffer chính để tránh bị đè
            memcpy(main_gps_buffer, (char*)rx_dma_buffer, Size);
            main_gps_buffer[Size] = '\0'; // Đóng chuỗi String

            gps_data_ready = 1; // Cắm cờ báo cho hàm Main biết đã có chuỗi mới
        }

        // QUAN TRỌNG: Mồi lại DMA để nó tiếp tục nhận chuỗi tiếp theo
        memset(rx_dma_buffer, 0, GPS_DMA_BUF_SIZE);
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer, GPS_DMA_BUF_SIZE);
    }
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
// Hàm chuyển đổi NMEA sang Decimal Degrees (Đã cập nhật dùng double)
double NMEA_To_DecimalDegrees(double nmea_coord, char dir) {
    int degrees = (int) (nmea_coord / 100.0);
    double minutes = nmea_coord - (degrees * 100.0);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    return decimal_degrees;
}

// Hàm xử lý chính gọi trong vòng lặp while(1)
void GPS_Process(GPS_Data_t *myGPS) {
    if (gps_data_ready == 1) {

        // 1. Tìm con trỏ (vị trí) bắt đầu của chuỗi GPGGA hoặc GNGGA trong mảng
        char *gga_ptr = strstr(main_gps_buffer, "$GPGGA");
        if (gga_ptr == NULL) {
            gga_ptr = strstr(main_gps_buffer, "$GNGGA");
        }

        // 2. Nếu tìm thấy chuỗi GGA trong mảng
        if (gga_ptr != NULL) {
            char tempBuf[20];
            double raw_lat = 0.0, raw_lon = 0.0;

            // LƯU Ý: Truyền con trỏ gga_ptr vào hàm get_field thay vì main_gps_buffer

            get_field(gga_ptr, 2, tempBuf);
            if (tempBuf[0] != '\0') raw_lat = strtod(tempBuf, NULL); // Dùng strtod cho double

            get_field(gga_ptr, 3, tempBuf);
            if (tempBuf[0] != '\0') myGPS->lat_dir = tempBuf[0];

            get_field(gga_ptr, 4, tempBuf);
            if (tempBuf[0] != '\0') raw_lon = strtod(tempBuf, NULL); // Dùng strtod cho double

            get_field(gga_ptr, 5, tempBuf);
            if (tempBuf[0] != '\0') myGPS->lon_dir = tempBuf[0];

            get_field(gga_ptr, 6, tempBuf);
            if (tempBuf[0] != '\0') myGPS->fix_quality = atoi(tempBuf);

            get_field(gga_ptr, 7, tempBuf);
            if (tempBuf[0] != '\0') myGPS->satellites = atoi(tempBuf);

            get_field(gga_ptr, 9, tempBuf);
            if (tempBuf[0] != '\0') myGPS->altitude = atof(tempBuf); // Độ cao dùng float vẫn ổn

            // Cập nhật tọa độ nếu GPS có tín hiệu vệ tinh
            if (raw_lat != 0.0 && raw_lon != 0.0 && myGPS->fix_quality > 0) {
                myGPS->latitude = NMEA_To_DecimalDegrees(raw_lat, myGPS->lat_dir);
                myGPS->longitude = NMEA_To_DecimalDegrees(raw_lon, myGPS->lon_dir);
                myGPS->ready = 1; // Tọa độ đã sẵn sàng đưa vào EKF/PID
            }
        }

        gps_data_ready = 0; // Hạ cờ sau khi xử lý xong toàn bộ cụm buffer
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Kiểm tra xem lỗi có xuất phát từ UART của GPS không
    if (huart->Instance == gps_huart->Instance) {
        // Hủy quá trình nhận hiện tại để dọn dẹp cờ lỗi
        HAL_UART_AbortReceive(huart);

        // Quan trọng: Mồi lại DMA để tiếp tục bắt tín hiệu
        HAL_UARTEx_ReceiveToIdle_DMA(gps_huart, rx_dma_buffer, GPS_DMA_BUF_SIZE);
    }
}
