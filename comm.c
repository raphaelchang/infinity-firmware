#include "comm.h"
#include "comm_can.h"
#include "comm_i2c.h"
#include "comm_uart.h"

#include "ch.h"
#include "hal.h"

static CommInterface comm = CAN;

static THD_WORKING_AREA(comm_update_wa, 1024);
static THD_FUNCTION(comm_update, arg) {
    (void)arg;

    chRegSetThreadName("Comm update");

    for(;;) {
        switch(comm) {
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
                break;
            case CUSTOM:
                break;

        // Forward commands to CAN and USB
        
        chThdSleepMilliseconds(10);
    }
}

void comm_init(void)
{
    switch(comm) {
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
            break;
        case CUSTOM:
            break;
    chThdCreateStatic(comm_update_wa, sizeof(comm_update_wa), NORMALPRIO, comm_update, NULL);
}