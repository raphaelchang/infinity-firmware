#include "encoder.h"
#include "comm_usb.h"

#include "ch.h"
#include "hal.h"

#include "as5x45.h"
#include "as5x47.h"
#include <math.h>

static EncoderType encoderType = AS5x47P;
static uint8_t polePairs = 7;
static float encoderZero = 26.0f;

void encoder_init(void)
{
    switch(encoderType) {
        case AS5045B:
        case AS5145B:
            as5x45_init(encoderType);
            break;
        case AS5x47P:
            as5x47_init();
            break;
        case HALL:
            break;
        case NONE:
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
    switch(encoderType) {
        case AS5045B:
        case AS5145B:
            deg = as5x45_get_angle() - encoderZero;
            edeg = fmod(deg, 360.0f / polePairs) * polePairs;
            break;
        case AS5x47P:
            deg = as5x47_get_angle() - encoderZero;
            edeg = fmod(deg, 360.0f / polePairs) * polePairs;
            break;
        case HALL:
            break;
        case NONE:
            break;
        default:
            break;
    }
    return edeg;
}