#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "datatypes.h"

void controller_init(void);
void controller_update(void);
void controller_print(void);
void controller_set_duty(float duty);
void controller_set_current(float current);
void controller_disable(void);
float controller_get_bus_voltage(void);
bool controller_encoder_zero(float current, float *zero, bool *inverted);
ControllerFault controller_get_fault(void);
ControllerState controller_get_state(void);

#endif
