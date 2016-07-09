#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"
#include "datatypes.h"

void encoder_init(void);
float encoder_get_angle(void);
float encoder_get_raw_angle(void);

#endif