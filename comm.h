#ifndef _COMM_H_
#define _COMM_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"

typedef enum
{
    CAN,
    I2C,
    UART,
    PPM,
    NUNCHUK,
    CUSTOM
} CommInterface;

void comm_init(void);

#endif