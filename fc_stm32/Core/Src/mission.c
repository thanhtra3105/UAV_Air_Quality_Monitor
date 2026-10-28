#include "mission.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "gps.h"
#include "flight_control.h"
#include "pos_hold.h"

MissionData_t current_mission;
static FlightControlState_t fc_state;

extern GPS_Data_t gps;
extern FlightMode_t flight_mode;

// Quản lý DMA UART nội bộ
static UART_HandleTypeDef *mission_huart = NULL;
static uint8_t dma_rx_buf[MISSION_BUFFER_SIZE];

// Bộ đệm nhận từng dòng JSON kết thúc bằng \n
static char line_buf[MAX_JSON_LINE_LEN];
static uint16_t line_idx = 0;

// Khởi tạo/Reset dữ liệu Mission
void Mission_Init(void) {
    current_mission.count = 0;
    current_mission.is_receiving = false;
    current_mission.is_ready = false;

    fc_state.next_wp_target = 0;
    fc_state.mission_started = false;

    line_idx = 0;
    memset(line_buf, 0, sizeof(line_buf));
}

// Khởi tạo phần cứng UART DMA với ngắt IDLE Line
void Mission_UART_Init(UART_HandleTypeDef *huart) {
    mission_huart = huart;
    Mission_Init();
    memset(dma_rx_buf, 0, sizeof(dma_rx_buf));

    if (mission_huart != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(mission_huart, dma_rx_buf, MISSION_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(mission_huart->hdmarx, DMA_IT_HT);
    }
}

// Hàm phụ trợ: Trích xuất giá trị dạng chuỗi theo key trong JSON
static bool get_json_string(const char *json, const char *key, char *out_val, size_t max_len) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    char *p = strstr(json, pattern);
    if (!p) return false;

    p = strchr(p + strlen(pattern), ':');
    if (!p) return false;

    p = strchr(p, '\"');
    if (!p) return false;
    p++;

    char *end = strchr(p, '\"');
    if (!end) return false;

    size_t len = (size_t)(end - p);
    if (len >= max_len) len = max_len - 1;
    strncpy(out_val, p, len);
    out_val[len] = '\0';
    return true;
}

// Hàm phụ trợ: Trích xuất giá trị số float / double theo key trong JSON
static bool get_json_double(const char *json, const char *key, double *out_val) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    char *p = strstr(json, pattern);
    if (!p) return false;

    p = strchr(p + strlen(pattern), ':');
    if (!p) return false;
    p++;

    while (*p == ' ' || *p == '\t') p++;
    *out_val = atof(p);
    return true;
}

// Phân tích danh sách điểm trong mảng "mission": [...]
static void parse_mission_array(const char *json) {
    char *p = strstr(json, "\"mission\"");
    if (!p) return;

    p = strchr(p, '[');
    if (!p) return;

    current_mission.count = 0;
    current_mission.is_receiving = true;
    current_mission.is_ready = false;

    while ((p = strchr(p, '{')) != NULL && current_mission.count < MAX_WAYPOINTS) {
        char *end_obj = strchr(p, '}');
        if (!end_obj) break;

        char wp_str[128];
        size_t node_len = (size_t)(end_obj - p + 1);
        if (node_len < sizeof(wp_str)) {
            strncpy(wp_str, p, node_len);
            wp_str[node_len] = '\0';

            double lat = 0.0, lon = 0.0, alt = 0.0, idx = 0.0;
            get_json_double(wp_str, "lat", &lat);
            get_json_double(wp_str, "lon", &lon);
            get_json_double(wp_str, "alt", &alt);
            get_json_double(wp_str, "index", &idx);

            current_mission.waypoints[current_mission.count].lat = lat;
            current_mission.waypoints[current_mission.count].lon = lon;
            current_mission.waypoints[current_mission.count].alt = (float)alt;
            current_mission.waypoints[current_mission.count].index = (uint16_t)idx;
            current_mission.count++;
        }
        p = end_obj + 1;
    }

    current_mission.is_receiving = false;
    if (current_mission.count > 0) {
        current_mission.is_ready = true;
    }
}

// Xử lý gói JSON và map sang FlightMode_t trong types.h
static void Mission_ProcessLine(char *line) {
    char cmd[32];
    if (!get_json_string(line, "cmd", cmd, sizeof(cmd))) {
        return;
    }

    // 1. Arm -> MODE_ARM
    if (strcmp(cmd, "arm") == 0) {
        fc_state.is_armed = true;
        fc_state.mode = MODE_ARM;
    }
    // 2. Disarm -> MODE_DISARM
    else if (strcmp(cmd, "disarm") == 0) {
        fc_state.is_armed = false;
        fc_state.mission_started = false;
        fc_state.mode = MODE_DISARM;
    }
    // 3. Takeoff -> MODE_TAKEOFF
    else if (strcmp(cmd, "takeoff") == 0) {
        double alt = 0.0;
        if (get_json_double(line, "alt", &alt)) {
            fc_state.target_alt = (float)alt;
        }
        fc_state.mode = MODE_TAKEOFF;
    }
    // 4. Upload mission (Waypoints)
    else if (strcmp(cmd, "upload_mission") == 0) {
        parse_mission_array(line);
    }
    // 5. Start mission -> MODE_NAVIGATION
    else if (strcmp(cmd, "start_mission") == 0) {
        fc_state.mission_started = true;
        fc_state.mode = MODE_NAVIGATION;
        double alt = 0.0;
        if (get_json_double(line, "alt", &alt)) {
            fc_state.target_alt = (float)alt;
        }
    }
    // 6. Stop mission -> Đưa về MODE_POSHOLD giữ vị trí
    else if (strcmp(cmd, "stop_mission") == 0) {
        fc_state.mission_started = false;
        fc_state.mode = MODE_POSHOLD;
    }
    // 7. Set altitude
    else if (strcmp(cmd, "set_altitude") == 0) {
        double alt = 0.0;
        if (get_json_double(line, "alt", &alt)) {
            fc_state.target_alt = (float)alt;
        }
    }
    // 8. RTL (Hạ cánh) -> MODE_LANDING
    else if (strcmp(cmd, "rtl") == 0) {
        fc_state.mode = MODE_LANDING;
        fc_state.mission_started = false;
    }
    // 9. Clear mission
    else if (strcmp(cmd, "clear_mission") == 0) {
        current_mission.count = 0;
        current_mission.is_ready = false;
    }
    // 10. Set mode (LOITER / GUIDED / AUTO)
    else if (strcmp(cmd, "set_mode") == 0) {
        char mode_str[16];
        if (get_json_string(line, "mode", mode_str, sizeof(mode_str))) {
            if (strcmp(mode_str, "LOITER") == 0)        fc_state.mode = MODE_POSHOLD;
            else if (strcmp(mode_str, "GUIDED") == 0)   fc_state.mode = MODE_POSHOLD;
            else if (strcmp(mode_str, "AUTO") == 0)     fc_state.mode = MODE_NAVIGATION;
            else if (strcmp(mode_str, "MANUAL") == 0)   fc_state.mode = MODE_MANUAL;
        }
    }
    // 11. Next waypoint
    else if (strcmp(cmd, "next_waypoint") == 0) {
        double wp_id = 0.0;
        if (get_json_double(line, "wp", &wp_id)) {
            fc_state.next_wp_target = (uint16_t)wp_id;
        }
    }
}

// Hàm đẩy luồng dữ liệu DMA vào để ghép dòng
void Mission_ParseChunk(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '\n') {
            line_buf[line_idx] = '\0';
            Mission_ProcessLine(line_buf);
            line_idx = 0;
        }
        else if (c != '\r') {
            if (line_idx < (MAX_JSON_LINE_LEN - 1)) {
                line_buf[line_idx++] = c;
            } else {
                line_idx = 0;
            }
        }
    }
}

// Xử lý sự kiện DMA hoàn thành một frame IDLE Line
void Mission_UartRxHandler(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == mission_huart && Size > 0) {
        Mission_ParseChunk(dma_rx_buf, Size);

        HAL_UARTEx_ReceiveToIdle_DMA(mission_huart, dma_rx_buf, MISSION_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(mission_huart->hdmarx, DMA_IT_HT);
    }
}

uint16_t Mission_GetCount(void) {
    return current_mission.count;
}

Waypoint_t Mission_GetWaypoint(uint16_t index) {
    Waypoint_t empty = {0, 0.0, 0.0, 0.0f};
    if (index < current_mission.count) {
        return current_mission.waypoints[index];
    }
    return empty;
}

bool Mission_IsReady(void) {
    return current_mission.is_ready;
}

const FlightControlState_t* FlightControl_GetState(void) {
    return &fc_state;
}

void FlightControl_SetMode(FlightMode_t mode) {
    fc_state.mode = mode;
}

void Mission_Complete(void) {
    fc_state.mode = MODE_POSHOLD;
    fc_state.mission_started = false;
}

// Khôi phục lại UART DMA khi bị lỗi đường truyền (Overrun, Noise, Framing)
void Mission_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == mission_huart && mission_huart != NULL) {
        // 1. Xóa các cờ lỗi phần cứng
        __HAL_UART_CLEAR_OREFLAG(mission_huart);
        __HAL_UART_CLEAR_NEFLAG(mission_huart);
        __HAL_UART_CLEAR_FEFLAG(mission_huart);

        // 2. Chỉ reset bộ đệm dòng JSON đang nhận dở (KHÔNG xóa current_mission hay fc_state)
        line_idx = 0;
        memset(line_buf, 0, sizeof(line_buf));

        // 3. Khởi động lại DMA để tiếp tục lắng nghe lệnh mới
        HAL_UARTEx_ReceiveToIdle_DMA(mission_huart, dma_rx_buf, MISSION_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(mission_huart->hdmarx, DMA_IT_HT);
    }
}

// 6. GỬI ACK CHUNG CHO TẤT CẢ CÁC LỆNH
// 		char ack_buf[128];
// 		snprintf(ack_buf, sizeof(ack_buf),
// 				"{\"type\":\"ack\",\"command\":\"%s\",\"ok\":true}\n",
// 				cmd->valuestring);
// //		HAL_UART_Transmit_DMA(mission_huart, (uint8_t*) ack_buf, strlen(ack_buf));

// 		if (mission_huart->gState != HAL_UART_STATE_READY) {
// 			HAL_UART_AbortTransmit(mission_huart);
// 		}

// 		// Bắt buộc dùng chế độ chặn (blocking) với timeout nhỏ (15ms).
// 		// Điều này đảm bảo khi gửi nhiều lệnh liên tiếp, lệnh 1 gửi xong hoàn toàn mới vòng sang xử lý và gửi ACK lệnh 2.
// 		HAL_UART_Transmit(mission_huart, (uint8_t*) ack_buf, strlen(ack_buf),
// 				15);


// ================= HÀM PHỤ TRỢ MAPPING CHUỖI =================
static const char* Get_Mode_String(FlightMode_t mode) {
    switch (mode) {
    case MODE_ARM:        return "ARM";
    case MODE_DISARM:     return "DISARM";
    case MODE_MANUAL:     return "MANUAL";
    case MODE_TAKEOFF:    return "TAKEOFF";
    case MODE_POSHOLD:    return "LOITER";
    case MODE_ALTHOLD:    return "ALTHOLD";
    case MODE_LANDING:    return "LANDING";
    case MODE_NAVIGATION: return "AUTO";
    default:              return "UNKNOWN";
    }
}

// ================= HÀM GỬI TELEMETRY QUA DMA CHO PI =================
void Mission_SendTelemetry(void) {
    if (mission_huart == NULL) return;

    // Chỉ kích hoạt truyền nếu kênh DMA UART đang rảnh
    if (mission_huart->gState == HAL_UART_STATE_READY) {
        static char telem_buf[384];

        float alt_m = kf_4d.altitude / 100.0f;          // Đổi từ cm sang mét
        float speed_ms = (float)gps.gSpeed / 1000.0f;   // Đổi từ mm/s sang m/s
        float voltage = txData.battery;
        bool is_armed = (flight_mode != MODE_DISARM);

        uint16_t total_wp = current_mission.count;
        int display_wp = 0;
        if (flight_mode == MODE_NAVIGATION && total_wp > 0) {
            display_wp = current_wp_index + 1;
        }

        int len = snprintf(telem_buf, sizeof(telem_buf),
                "{\"telemetry\":{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.2f,\"speed\":%.2f,\"heading\":%.1f,\"satellites\":%d,"
                "\"pitch\":%.1f,\"roll\":%.1f,\"yaw\":%.1f,\"voltage\":%.2f,\"armed\":%s,\"mode\":\"%s\",\"current_wp\":%d}}\n",
                gps.latitude, gps.longitude, alt_m, speed_ms, yaw, (int)gps.numSV,
                pitch, roll, yaw, voltage,
                is_armed ? "true" : "false",
                Get_Mode_String(flight_mode),
                display_wp);

        if (len > 0 && len < sizeof(telem_buf)) {
            HAL_UART_Transmit_DMA(mission_huart, (uint8_t*) telem_buf, (uint16_t) len);
        }
    }
}
