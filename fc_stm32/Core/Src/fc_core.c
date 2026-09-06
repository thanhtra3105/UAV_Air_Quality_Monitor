/**
 * @file fc_core.c
 * @brief Flight Controller Core & Main Loop Scheduler Implementation
 */

#include "fc_core.h"
#include "dwt.h"
#include "serial.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart7;
extern TIM_HandleTypeDef  htim6;

// Central Vehicle State instance
VehicleState_t g_veh;

// Buffer for ESP32 Mission DMA chunk reception
uint8_t rx_mission_buffer[MISSION_BUFFER_SIZE];

void FlightController_Init(void) {
    // 0. Initialize Vehicle State default values
    memset(&g_veh, 0, sizeof(VehicleState_t));
    g_veh.throttle = 1000;
    g_veh.throttle_hover = 1000;

    // 1. Start TIM6 IT for timer tick
    HAL_TIM_Base_Start_IT(&htim6);

    // 2. Initialize Hardware Drivers & Subsystems
    RC_Init();
    Motors_Init();
    PosEstimator_Init();
    AHRS_Init(&g_veh);
    AttitudeControl_Init();
    FlightModes_Init();
    Telemetry_Init();

    // 3. Initialize Mission subsystem and start UART7 DMA reception
    Mission_Init();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rx_mission_buffer, MISSION_BUFFER_SIZE);

    // 4. Initialize RC Receiver and wait for zero throttle
    RC_WaitForZeroThrottle(&g_veh);

    Serial_printf(&huart1, "[OK] START FLIGHT CONTROLLER\r\n");
}

void FlightController_Run(void) {
    static uint32_t loop_count = 0;
    static uint32_t ina219_timer = 0;
    static uint32_t esp_timer = 0;

    uint32_t start = DWT_GetMicros();
    float dt = 0.002f; // 500 Hz fast loop

    // =========================================================================
    // 1. FAST LOOP (500 Hz): RC, State Estimation, Rate PID, Motors
    // =========================================================================
    RC_Process(&g_veh);

    // Yaw hold initialization on throttle push
    if (!g_veh.yaw_hold_init && g_veh.throttle > 1030) {
        g_veh.target_yaw = g_veh.yaw;
        g_veh.yaw_hold_init = true;
    }
    if (g_veh.throttle < 1030) {
        g_veh.yaw_hold_init = false;
        g_veh.target_yaw = g_veh.yaw;
    }

    // Sensor acquisition & Attitude estimation
    AHRS_ReadIMU(&g_veh);
    AHRS_UpdateAttitude(&g_veh, dt);

    // High rate GPS Kalman prediction
    PosEstimator_UpdateGPS(&g_veh, dt);

    // =========================================================================
    // 2. MEDIUM LOOP (250 Hz): Flight Modes & Outer Angle PID
    // =========================================================================
    if ((loop_count & 1) == 0) {
        float dt_250hz = dt * 2.0f;

        // Altitude & Position hold setpoint updates
        FlightModes_Update(&g_veh, dt_250hz);

        // Outer angle PID loop -> Target angular rates
        AttitudeControl_AngleLoop(&g_veh, dt_250hz);

        // Optical flow / TOF packet processing
        MTF01_Update(&mtf_data);
    }

    // =========================================================================
    // 3. INNER RATE LOOP & MOTOR MIXER (500 Hz)
    // =========================================================================
    AttitudeControl_RateLoop(&g_veh, dt);
    Motors_Mixer(&g_veh);

    // =========================================================================
    // 4. SLOW TASKS: GPS (~10Hz), Telemetry (~2Hz/1Hz)
    // =========================================================================
    if ((loop_count % 10) == 0) {
        GPS_Process(&gps);
    }

    uint32_t current_time = DWT_GetMicros();

    // 2 Hz task: Battery measurement
    if (current_time - ina219_timer > 500000) {
        Telemetry_UpdateBattery(&g_veh);
        ina219_timer = current_time;
    }

    // 1 Hz task: ESP32 telemetry & LED indicator
    if (current_time - esp_timer > 1000000) {
        Telemetry_SendESP32(&g_veh);
        Telemetry_UpdateLEDs(&g_veh);
        esp_timer = current_time;
    }

    loop_count++;

    // Maintain precise 2000 microsecond (500 Hz) cycle
    while ((DWT_GetMicros() - start) < 2000) {
        // Spin wait
    }
}
