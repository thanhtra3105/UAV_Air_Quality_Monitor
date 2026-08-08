/*
 * pid_controller.c
 */

#include "pid_controller.h"

static float constrainFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void PID_SetIntegralLimits(PIDController_t *pid, float iMin, float iMax)
{
    pid->iMin = iMin;
    pid->iMax = iMax;
}

void PID_Init(PIDController_t *pid,
              float kp,
              float ki,
              float kd,
              float alpha)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->preE = 0.0f;
    pid->preI = 0.0f;
    pid->preD = 0.0f;

    pid->alphaD = alpha;

    pid->iMin = -200.0f;
    pid->iMax = 200.0f;
}

float PID_Calculate(PIDController_t *pid,
                    float error,
                    float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    /* P */
    float P = pid->kp * error;

    /* I (Trapezoidal Integration) */
    pid->preI += pid->ki * (error + pid->preE) * dt * 0.5f;
    pid->preI = constrainFloat(pid->preI,
                               pid->iMin,
                               pid->iMax);

    float I = pid->preI;

    /* D */
    float rawD = pid->kd * (error - pid->preE) / dt;

    /* LPF */
    float D = pid->alphaD * rawD +
              (1.0f - pid->alphaD) * pid->preD;

    pid->preE = error;
    pid->preD = D;

    return P + I + D;
}

void PID_Reset(PIDController_t *pid)
{
    pid->preE = 0.0f;
    pid->preI = 0.0f;
    pid->preD = 0.0f;
}

void PID_SetGain(PIDController_t *pid,
                 float kp,
                 float ki,
                 float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
