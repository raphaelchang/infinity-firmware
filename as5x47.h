#ifndef _AS5X47_H_
#define _AS5X47_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"

void as5x47_init(void);
double as5x47_get_angle(void);
uint16_t as5x47_get_raw_position(void);

#endif