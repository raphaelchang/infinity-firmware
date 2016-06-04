#include "encoder.h"

#include "ch.h"
#include "hal.h"

static EncoderType encoderType = AS5045B;

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
double encoder_get_angle(void)
{
    double edeg = 0;
    switch(encoderType) {
        case AS5045B:
        case AS5145B:
            double deg = as5x45_get_angle();
            // Process pole pairs
            break;
        case AS5x47P:
            double deg = as5x47_get_angle();
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