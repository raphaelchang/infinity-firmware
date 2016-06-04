#include "as5x47.h"

#include "ch.h"
#include "hal.h"

static const SPIConfig encoderSPI =
{
    NULL,
    GPIOA,
    GPIOA_LRCK,
    SPI_CR1_BR_0
};

void as5x47_init(void)
{
    spiStart(&SPID1, &encoderSPI);
}

double as5x47_get_angle(void)
{

}

uint16_t as5x47_get_raw_position(void)
{

}