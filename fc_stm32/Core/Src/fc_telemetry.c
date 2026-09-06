/**
 * @file fc_telemetry.c
 * @brief Telemetry & Ground Control Station Communication Implementation
 */

#include "fc_telemetry.h"
#include "ina219.h"
#include "gps.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart7;
extern GPS_Data_t gps;

void Telemetry_Init(void) {
    INA219_Init(&hi2c2);
}

void Telemetry_UpdateBattery(VehicleState_t *veh) {
    if (veh == NULL) return;

    float voltage = INA219_Read_Bus_Voltage(&hi2c2);
    if (voltage >= 0.0f) {
        veh->battery_voltage = voltage;
        veh->telemetry.battery = voltage;
    }
}

void Telemetry_SendESP32(VehicleState_t *veh) {
    (void)veh;
    static char tx_buf[64];
    int len = snprintf(tx_buf, sizeof(tx_buf), "POS,%.7f,%.7f\n", gps.latitude, gps.longitude);
    if (len > 0) {
        HAL_UART_Transmit_DMA(&huart7, (uint8_t*)tx_buf, (uint16_t)len);
    }
}

void Telemetry_UpdateLEDs(const VehicleState_t *veh) {
    if (veh == NULL) return;

    if (gps.fixType == 3 && gps.numSV > 20 && veh->mission_running) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}
