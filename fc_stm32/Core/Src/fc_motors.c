/**
 * @file fc_motors.c
 * @brief Actuator & Quad-X Motor Mixer Implementation
 */

#include "fc_motors.h"
#include "types.h"

extern TIM_HandleTypeDef htim3;

void Motors_Init(void) {
    HAL_TIM_PWM_Init(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    Motors_DisarmAll();
}

void Motors_Write(uint8_t channel, int16_t pwm_us) {
    __HAL_TIM_SET_COMPARE(&htim3, channel, pwm_us);
}

void Motors_DisarmAll(void) {
    Motors_Write(M1, 1000);
    Motors_Write(M2, 1000);
    Motors_Write(M3, 1000);
    Motors_Write(M4, 1000);
}

void Motors_Mixer(VehicleState_t *veh) {
    if (veh == NULL) return;

    int16_t throttle = veh->throttle;
    float pid_p      = veh->pid_p;
    float pid_r      = veh->pid_r;
    float pid_y      = veh->pid_y;
    float pid_alt    = veh->pid_alt;

    // Quad-X mixer
    int m1 = (int)((float)throttle - pid_p - pid_r - pid_y + pid_alt);
    int m2 = (int)((float)throttle + pid_p - pid_r + pid_y + pid_alt);
    int m3 = (int)((float)throttle + pid_p + pid_r - pid_y + pid_alt);
    int m4 = (int)((float)throttle - pid_p + pid_r + pid_y + pid_alt);

    // Save for telemetry / debug
    veh->motor_pwm[0] = (int16_t)m1;
    veh->motor_pwm[1] = (int16_t)m2;
    veh->motor_pwm[2] = (int16_t)m3;
    veh->motor_pwm[3] = (int16_t)m4;

    if (throttle > 1040) {
        Motors_Write(M1, m1);
        Motors_Write(M2, m2);
        Motors_Write(M3, m3);
        Motors_Write(M4, m4);
    } else {
        Motors_DisarmAll();
    }
}
