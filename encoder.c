#include "encoder.h"
#include "comm_usb.h"

#include "ch.h"
#include "hal.h"

#include "as5x45.h"
#include "as5x47.h"
#include <math.h>
#include "config.h"

static volatile Config *config;

void encoder_init(void)
{
    config = config_get_configuration();
    switch(config->encoderType) {
        case AS5045B:
        case AS5145B:
            as5x45_init(config->encoderType);
            break;
        case AS5x47P:
            as5x47_init();
            break;
        case HALL:
            break;
        case SENSORLESS:
            break;
        default:
            break;
    }
}

/* Returns rotor electrical degrees */
float encoder_get_angle(void)
{
    float edeg = 0;
    float deg = 0;
    switch(config->encoderType) {
        case AS5045B:
        case AS5145B:
            deg = as5x45_get_angle() - config->encoderZero;
            if (deg < 0)
            {
                deg += 360.0;
            }
            edeg = fmod(deg, 360.0f / config->polePairs) * config->polePairs;
            break;
        case AS5x47P:
            deg = as5x47_get_angle() - config->encoderZero;
            if (deg < 0)
            {
                deg += 360.0;
            }
            edeg = fmod(deg, 360.0f / config->polePairs) * config->polePairs;
            break;
        case HALL:
            break;
        case SENSORLESS:
            break;
        default:
            break;
    }
    return edeg;
}