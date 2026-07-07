/*
 * pid_controller.h
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include "stm32h5xx_hal.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

    float preE;
    float preI;

    // Low-pass filter cho D
    float preD;
    float alphaD;

    // Giới hạn tích phân
    float iMin;
    float iMax;

} PIDController_t;

/* Khởi tạo PID */
void PID_Init(PIDController_t *pid,
              float kp,
              float ki,
              float kd,
              float alpha);

/* Tính PID */
float PID_Calculate(PIDController_t *pid,
                    float error,
                    float dt);

/* Reset trạng thái */
void PID_Reset(PIDController_t *pid);

/* Thay đổi hệ số PID */
void PID_SetGain(PIDController_t *pid,
                 float kp,
                 float ki,
                 float kd);

#endif
