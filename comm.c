#include "comm.h"
#include "comm_can.h"
#include "comm_i2c.h"
#include "comm_uart.h"
#include "comm_nunchuk.h"
#include "comm_usb.h"

#include "ch.h"
#include "hal.h"
#include "config.h"

static volatile Config *config;
static volatile bool usb_override = false;

static THD_WORKING_AREA(comm_update_wa, 1024);
static THD_FUNCTION(comm_update, arg) {
    (void)arg;

    chRegSetThreadName("Comm update");

    switch(config->commInterface) {
        case CAN:
            comm_can_init();
            break;
        case I2C:
            comm_i2c_init();
            break;
        case UART:
            comm_uart_init();
            break;
        case PPM:
            break;
        case NUNCHUK:
            comm_nunchuk_init();
            break;
        case NRF:
            break;
        case CUSTOM:
            break;
        default:
            break;
    }
    for(;;) {
        if (!usb_override)
        {
            switch(config->commInterface) {
                case CAN:
                    comm_can_update();
                    break;
                case I2C:
                    comm_i2c_update();
                    break;
                case UART:
                    comm_uart_update();
                    break;
                case PPM:
                    break;
                case NUNCHUK:
                    comm_nunchuk_update();
                    break;
                case NRF:
                    break;
                case CUSTOM:
                    break;
                default:
                    break;
            }
        }
        if (!comm_usb_serial_is_active())
        {
            usb_override = false;
        }

        // Forward commands to CAN and USB
        
        chThdSleepMilliseconds(10);
    }
}

void comm_init(void)
{
    config = config_get_configuration();
    chThdCreateStatic(comm_update_wa, sizeof(comm_update_wa), NORMALPRIO, comm_update, NULL);
}

void comm_set_usb_override(bool enable)
{
    usb_override = enable;
}

bool comm_get_usb_override(void)
{
    return usb_override;
}
