#ifndef _AS5X45_H_
#define _AS5X45_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"
#include "encoder.h"

void as5x45_init(EncoderType type);
double as5x45_get_angle(void);
uint16_t as5x45_get_raw_position(void);

#endif