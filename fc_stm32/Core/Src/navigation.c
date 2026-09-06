/*
 * navigation.c
 *
 *  Created on: Aug 24, 2026
 *      Author: lethanhtra
 */
#include "navigation.h"

float calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;

    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    double R = 6371000.0; // Bán kính Trái Đất (mét)

    return (float)(R * c);
}

// Tính góc phương vị (Bearing) từ điểm 1 đến điểm 2 (trả về -180 đến 180 độ)
float calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double dLon_rad = (lon2 - lon1) * DEG_TO_RAD;

    double y = sin(dLon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);

    double bearing = atan2(y, x) * RAD_TO_DEG;

    // Đưa về dải -180 đến 180 để khớp với IMU của bạn
    if (bearing > 180.0f) bearing -= 360.0f;
    if (bearing < -180.0f) bearing += 360.0f;

    return (float)bearing;
}
