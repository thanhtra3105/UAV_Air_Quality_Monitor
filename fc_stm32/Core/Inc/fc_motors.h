/**
 * @file fc_motors.h
 * @brief Actuator & Quad-X Motor Mixer
 */

#ifndef FC_MOTORS_H_
#define FC_MOTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "fc_state.h"

void Motors_Init(void);
void Motors_Write(uint8_t channel, int16_t pwm_us);
void Motors_Mixer(VehicleState_t *veh);
void Motors_DisarmAll(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_MOTORS_H_ */
