#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

typedef enum
{
	STOPPED,
	RUNNING,
	ZEROING
} ControllerState;

typedef enum
{
	NO_FAULT,
	OVERCURRENT,
	OVERVOLTAGE,
	UNDERVOLTAGE,
	TEMPERATURE
} ControllerFault;

void controller_init(void);
void controller_update(void);
void controller_apply_zsm(float *a, float *b, float *c);
void controller_print_adc(void);
ControllerFault controller_get_fault(void);

#endif