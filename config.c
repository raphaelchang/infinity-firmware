#include "config.h"
#include "ch.h"
#include "hal.h"

static volatile Config config;

void config_init(void)
{
    config.encoderType = AS5x47P;
    config.commInterface = NUNCHUK;
    config.zsmMode = BOTTOM_CLAMP;
    config.polePairs = 7;
    config.encoderZero = 119.5f;
    config.encoderInverted = true;
    config.pwmFrequency = 20000.0;
    config.currentKp = 0.1;
    config.currentKi = 50.0;
    config.maxDuty = 0.8;
    config.maxCurrent = 10.0;
}

Config* config_get_configuration(void)
{
    return &config;
}
