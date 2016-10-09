#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "datatypes.h"

void controller_init(void);
void controller_update(void);
void controller_print(void);
void controller_set_duty(float duty);
void controller_set_current(float current);
void controller_set_speed(float speed);
void controller_disable(void);
float controller_get_bus_voltage(void);
bool controller_encoder_zero(float current, float *zero, bool *inverted);
float controller_measure_resistance(float current, uint16_t samples);
ControllerFault controller_get_fault(void);
ControllerState controller_get_state(void);
float controller_get_command_current(void);
float controller_get_erpm(void);
float controller_get_current_q(void);
float controller_get_current_d(void);
float controller_get_temperature(void);

#endif
