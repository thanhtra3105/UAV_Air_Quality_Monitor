#include "mission.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

MissionData_t current_mission;

// Bộ đệm phụ để ghép nối các ký tự thành 1 dòng hoàn chỉnh
#define MAX_LINE_LEN 64
static char line_buf[MAX_LINE_LEN];
static uint16_t line_idx = 0;

void Mission_Init(void) {
    current_mission.count = 0;
    current_mission.is_receiving = false;
    current_mission.is_ready = false;
    line_idx = 0;
    memset(line_buf, 0, MAX_LINE_LEN);
}

// Hàm xử lý 1 dòng dữ liệu (khi đã nhận đủ dấu \n)
static void Mission_ProcessLine(char *line) {
    if (strncmp(line, "MISSION_START", 13) == 0) {
        current_mission.count = 0;
        current_mission.is_receiving = true;
        current_mission.is_ready = false;
        return;
    }

    if (strncmp(line, "MISSION_END", 11) == 0) {
        if (current_mission.is_receiving) {
            current_mission.is_receiving = false;
            if (current_mission.count > 0) {
                current_mission.is_ready = true;
            }
        }
        return;
    }

    // Nếu đang trong quá trình nhận và dòng bắt đầu bằng "WP,"
    if (current_mission.is_receiving && strncmp(line, "WP,", 3) == 0) {
        // Parse chuỗi dạng: WP,0,16.054400,108.202200
        char *token = strtok(line, ","); // Lấy "WP" (bỏ qua)

        if (token != NULL) {
            token = strtok(NULL, ","); // Lấy ID (không cần lưu trữ trừ khi bạn muốn check tuần tự)
        }

        if (token != NULL) {
            token = strtok(NULL, ","); // Lấy Vĩ độ (Latitude)
            if (token != NULL && current_mission.count < MAX_WAYPOINTS) {
                double lat = atof(token);

                token = strtok(NULL, ","); // Lấy Kinh độ (Longitude)
                if (token != NULL) {
                    double lon = atof(token);

                    // Lưu vào mảng
                    current_mission.waypoints[current_mission.count].lat = lat;
                    current_mission.waypoints[current_mission.count].lon = lon;
                    current_mission.count++;
                }
            }
        }
    }
}

// Hàm đẩy luồng dữ liệu thô từ DMA vào, tự động tách dòng
void Mission_ParseChunk(uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        char c = (char)data[i];

        if (c == '\n') {
            line_buf[line_idx] = '\0'; // Kết thúc chuỗi C standard
            Mission_ProcessLine(line_buf);
            line_idx = 0; // Reset bộ đệm dòng để đọc dòng tiếp theo
        }
        else if (c != '\r' && line_idx < (MAX_LINE_LEN - 1)) {
            line_buf[line_idx++] = c;
        }
    }
}

// Các hàm Getter cho các module khác gọi (Flight Controller, Navigation...)
uint16_t Mission_GetCount(void) {
    return current_mission.count;
}

Waypoint_t Mission_GetWaypoint(uint16_t index) {
    Waypoint_t empty = {0.0, 0.0};
    if (index < current_mission.count) {
        return current_mission.waypoints[index];
    }
    return empty; // Trả về 0 nếu gọi sai index
}

bool Mission_IsReady(void) {
    return current_mission.is_ready;
}
