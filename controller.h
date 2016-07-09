#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "datatypes.h"

void controller_init(void);
void controller_update(void);
void controller_apply_zsm(volatile float *a, volatile float *b, volatile float *c);
void controller_print(void);
ControllerFault controller_get_fault(void);
ControllerState controller_get_state(void);

#endif