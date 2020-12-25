#ifndef _AS5X47_H_
#define _AS5X47_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"

void as5x47_init(void);
float as5x47_get_angle(void);
void as5x47_update_raw_position(void);
uint16_t as5x47_get_raw_position(void);
float as5x47_get_last_angle(void);

#endif
