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

/* BỘ ĐỆM RAM GHI CHUYẾN BAY: 3600 mẫu * 92 bytes = ~331 KB RAM */
static FlightLogData_t ram_log_buffer[MAX_RAM_LOG_SAMPLES];
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
 * @brief  Khởi tạo hệ thống RAM Blackbox
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

    Serial_printf(&huart1, "[LOG] RAM Blackbox san sang! (~330KB RAM, bat dau ghi khi throttle >= 1400)\r\n");
    return 0;
}

/**
 * @brief  Ghi mẫu trực tiếp vào RAM (Chỉ tốn 0.05 us)
 */
void Log_Write_Custom(const FlightLogData_t *data)
{
    if (data == NULL || is_saving_active || has_saved_to_sd) {
        return;
    }

    // 1. Chỉ bắt đầu kích hoạt ghi khi cần ga được đẩy lên >= 1400
    if (!is_recording) {
        if (throttle >= LOG_THROTTLE_START) {
            is_recording = true;
//            Serial_printf(&huart1, "[LOG] Ga >= %d -> Bat dau ghi vao RAM!\r\n", LOG_THROTTLE_START);
        } else {
            return; // Chưa đủ ga, không ghi
        }
    }

    // 2. Chép mẫu vào RAM nếu còn chỗ (Khi đầy 3600 mẫu thì tự động ngừng, không làm tràn RAM)
    if (ram_log_count < MAX_RAM_LOG_SAMPLES) {
        ram_log_buffer[ram_log_count] = *data;
        ram_log_count++;
    }
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

    // 4. Ghi tiêu đề các cột chuẩn CSV (Header) cho Paper Benchmark
    const char header[] = "time_ms,"
                          "target_roll,roll_raw,roll_cf,roll_kf,"
                          "target_pitch,pitch_raw,pitch_cf,pitch_kf,"
                          "alt_tof,alt_baro,alt_kf_2d,alt_kf,vz_raw,vz_kf_2d,vz_kf,az_raw,"
                          "acc_bias,baro_bias,"
                          "pos_x_raw,pos_x_kf,pos_y_raw,pos_y_kf,"
                          "vel_x_raw,vel_x_kf,vel_y_raw,vel_y_kf\r\n";

    fx_file_write(&log_file, (void *)header, strlen(header));

    // 5. Xả tuần tự các mẫu từ RAM xuống buffer rồi ghi ra thẻ SD
    uint16_t chunk_len = 0;
    for (uint16_t i = 0; i < ram_log_count; i++) {
        FlightLogData_t *p = &ram_log_buffer[i];
        char line[280];
        int len = snprintf(line, sizeof(line),
            "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
            (unsigned long)p->time_ms,
            p->target_roll, p->roll_raw, p->roll_cf, p->roll_kf,
            p->target_pitch, p->pitch_raw, p->pitch_cf, p->pitch_kf,
            p->alt_raw_tof, p->alt_raw_baro, p->alt_kf_2d, p->alt_kf,
            p->vz_raw, p->vz_kf_2d, p->vz_kf, p->az_raw,
            p->acc_bias, p->baro_bias,
            p->pos_x_raw, p->pos_x_kf, p->pos_y_raw, p->pos_y_kf,
            p->vel_x_raw, p->vel_x_kf, p->vel_y_raw, p->vel_y_kf);

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
