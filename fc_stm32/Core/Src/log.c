/*
 * log.c
 *
 *  Author: Antigravity - RAM Flight Blackbox Logger (Zero DT Jitter)
 */

#include "log.h"
#include "app_filex.h"
#include "fx_api.h"
#include "fx_stm32_sd_driver.h"
#include "flight_control.h"
#include "serial.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern int throttle;
extern GPS_Data_t gps;
extern uint8_t gps_new_data_flag;

/* BỘ ĐỆM RAM GHI GIA TỐC 500Hz + GPS: 12,000 mẫu * 32 bytes = 384 KB RAM */
static AccelLog_t      ram_log_buffer[MAX_RAM_LOG_SAMPLES];
static uint16_t        ram_log_count = 0;
static bool            is_recording = false;
static bool            has_saved_to_sd = false;
static bool            is_saving_active = false;

/* Biến FileX nội bộ */
static FX_MEDIA sdio_disk;
static FX_FILE  log_file;
ALIGN_32BYTES(static uint8_t media_memory[FX_STM32_SD_DEFAULT_SECTOR_SIZE]);
ALIGN_32BYTES(static char    sd_chunk_buffer[1024]);

/**
 * @brief  Khởi tạo hệ thống RAM Blackbox 500Hz
 */
uint8_t Log_Init(void)
{
    ram_log_count = 0;
    is_recording = false;
    has_saved_to_sd = false;
    is_saving_active = false;

    // Đảm bảo ngắt SDMMC1 được bật sẵn sàng cho lúc hạ cánh lưu file
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    Serial_printf(&huart1, "[LOG] RAM Blackbox san sang! (12.000 mau Accel 500Hz + GPS ~ 24s bay, bat dau khi throttle >= %d)\r\n", LOG_THROTTLE_START);
    return 0;
}

/**
 * @brief  Ghi chi tiết mẫu trực tiếp vào RAM
 */
void Log_Write_Data(uint32_t time_us, float ax, float ay, float velN, float velE, float hAcc, float sAcc, uint8_t gps_updated)
{
    if (is_saving_active || has_saved_to_sd) {
        return;
    }

    // 1. Chỉ bắt đầu kích hoạt ghi khi cần ga được đẩy lên >= LOG_THROTTLE_START
    if (!is_recording) {
        if (throttle >= LOG_THROTTLE_START) {
            is_recording = true;
        } else {
            return; // Chưa đủ ga, không ghi
        }
    }

    // 2. Chép mẫu vào RAM nếu còn chỗ (Khi đầy 12000 mẫu thì tự động ngừng, không tràn RAM)
    if (ram_log_count < MAX_RAM_LOG_SAMPLES) {
        ram_log_buffer[ram_log_count].time_us = time_us;
        ram_log_buffer[ram_log_count].ax_earth = ax;
        ram_log_buffer[ram_log_count].ay_earth = ay;
        ram_log_buffer[ram_log_count].velN = velN;
        ram_log_buffer[ram_log_count].velE = velE;
        ram_log_buffer[ram_log_count].hAcc = hAcc;
        ram_log_buffer[ram_log_count].sAcc = sAcc;
        ram_log_buffer[ram_log_count].gps_updated = gps_updated;
        ram_log_count++;
    }
}

/**
 * @brief  Ghi đầy đủ Accel 500Hz và trích xuất dữ liệu GPS sang hệ đơn vị SI (m, m/s)
 */
void Log_Write_Flight(uint32_t time_us, float ax, float ay, const GPS_Data_t *myGPS)
{
    float velN = 0.0f;
    float velE = 0.0f;
    float hAcc = 0.0f;
    float sAcc = 0.0f;
    uint8_t updated = 0;

    if (myGPS != NULL) {
        velN = (float)myGPS->velN * 0.001f; // Chuyển từ mm/s sang m/s
        velE = (float)myGPS->velE * 0.001f; // Chuyển từ mm/s sang m/s
        hAcc = (float)myGPS->hAcc * 0.001f; // Chuyển từ mm sang m
        sAcc = (float)myGPS->sAcc * 0.001f; // Chuyển từ mm/s sang m/s
    }

    updated = gps_new_data_flag;
    gps_new_data_flag = 0; // Xóa cờ sau khi đã ghi nhận

    Log_Write_Data(time_us, ax, ay, velN, velE, hAcc, sAcc, updated);
}

/**
 * @brief  Ghi gia tốc trực tiếp vào RAM (Tự động lấy thêm dữ liệu GPS toàn cục)
 */
void Log_Write_Accel(uint32_t time_us, float ax, float ay)
{
    Log_Write_Flight(time_us, ax, ay, &gps);
}

/**
 * @brief  Tự động phát hiện hạ cánh để xả toàn bộ RAM xuống thẻ MicroSD
 */
void Log_Check_Auto_Save(void)
{
    // Nếu đã từng bay (đã kích hoạt ghi) và nay kéo ga về < 1030 (hạ cánh)
    if (is_recording && (throttle < LOG_THROTTLE_LANDED) && !has_saved_to_sd && (ram_log_count >= 50))
    {
        Log_Save_RAM_To_SD();
        has_saved_to_sd = true;
        is_recording = false;
    }
}

/**
 * @brief  Xả toàn bộ dữ liệu từ RAM xuống thẻ MicroSD thành file CSV
 *         (Chỉ chạy KHI ĐÃ HẠ CÁNH, không ảnh hưởng gì tới vòng lặp bay 500Hz)
 */
void Log_Save_RAM_To_SD(void)
{
    UINT status;
    char filename[32];

    if (ram_log_count == 0 || is_saving_active) {
        return;
    }

    is_saving_active = true;
    Serial_printf(&huart1, "\r\n[LOG] Ha canh thanh cong! Dang mo the SD va luu %u mau tu RAM...\r\n", ram_log_count);

    // 1. Mở thẻ nhớ FileX
    status = fx_media_open(&sdio_disk, "STM32_SD_DISK", fx_stm32_sd_driver,
                           (VOID *)FX_STM32_SD_INSTANCE, media_memory, sizeof(media_memory));
    if (status != FX_SUCCESS) {
        Serial_printf(&huart1, "[LOG ERROR] fx_media_open that bai! Ma loi: 0x%02X\r\n", status);
        is_saving_active = false;
        return;
    }

    // 2. Tự động tìm tên file mới theo thứ tự: flight_001.csv, flight_002.csv...
    int file_idx = 1;
    for (; file_idx <= 999; file_idx++) {
        snprintf(filename, sizeof(filename), "flight_%03d.csv", file_idx);
        status = fx_file_create(&sdio_disk, filename);
        if (status == FX_SUCCESS) {
            break;
        }
        if (status != FX_ALREADY_CREATED) {
            Serial_printf(&huart1, "[LOG ERROR] fx_file_create that bai! Ma loi: 0x%02X\r\n", status);
            fx_media_close(&sdio_disk);
            is_saving_active = false;
            return;
        }
    }

    // 3. Mở file để ghi
    status = fx_file_open(&sdio_disk, &log_file, filename, FX_OPEN_FOR_WRITE);
    if (status != FX_SUCCESS) {
        Serial_printf(&huart1, "[LOG ERROR] Khong the mo file %s! Ma loi: 0x%02X\r\n", filename, status);
        fx_media_close(&sdio_disk);
        is_saving_active = false;
        return;
    }

    // 4. Ghi tiêu đề các cột chuẩn CSV (Header) cho Accel + GPS Benchmark
    const char header[] = "time_us,ax_earth,ay_earth,velN,velE,hAcc,sAcc,gps_updated\r\n";

    fx_file_write(&log_file, (void *)header, strlen(header));

    // 5. Xả tuần tự các mẫu từ RAM xuống buffer rồi ghi ra thẻ SD
    uint16_t chunk_len = 0;
    for (uint16_t i = 0; i < ram_log_count; i++) {
        AccelLog_t *p = &ram_log_buffer[i];
        char line[96];
        int len = snprintf(line, sizeof(line),
            "%lu,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%u\r\n",
            (unsigned long)p->time_us,
            p->ax_earth,
            p->ay_earth,
            p->velN,
            p->velE,
            p->hAcc,
            p->sAcc,
            (unsigned int)p->gps_updated);

        if (len > 0) {
            // Nếu buffer gần đầy -> ghi khối xuống thẻ SD
            if ((chunk_len + len) >= sizeof(sd_chunk_buffer)) {
                fx_file_write(&log_file, sd_chunk_buffer, chunk_len);
                chunk_len = 0;
            }
            memcpy(&sd_chunk_buffer[chunk_len], line, len);
            chunk_len += (uint16_t)len;
        }
    }

    // Ghi phần còn lại trong buffer
    if (chunk_len > 0) {
        fx_file_write(&log_file, sd_chunk_buffer, chunk_len);
    }

    // 6. Đóng file và đồng bộ an toàn
    fx_media_flush(&sdio_disk);
    fx_file_close(&log_file);
    fx_media_close(&sdio_disk);

    Serial_printf(&huart1, "[LOG OK] Da luu xong %u mau vao file: %s! Rut the doc tren PC binh thuong.\r\n",
                  ram_log_count, filename);
    is_saving_active = false;
}

uint16_t Log_GetSampleCount(void) { return ram_log_count; }
bool Log_IsLogging(void)           { return is_recording; }
bool Log_IsSaving(void)            { return is_saving_active; }
