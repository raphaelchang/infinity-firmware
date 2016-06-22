#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"

typedef enum
{
    AS5045B, // SSI, 12-bit angular, 10-bit incremental
    AS5145B, // SSI, 12-bit angular, 12-bit incremental
    AS5x47P, // SPI, 14-bit angular, 12-bit incremental
    HALL,
    NONE
} EncoderType;

void encoder_init(void);
float encoder_get_angle(void);

#endif