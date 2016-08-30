#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "comm.h"
#include "ws2812b.h"
#include "encoder.h"
#include "controller.h"
#include "chprintf.h"
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "gpio.h"
#include "config.h"
#include "packet.h"
#include "scope.h"

static THD_WORKING_AREA(led_update_wa, 1024);
static THD_FUNCTION(led_update, arg) {
    (void)arg;

    chRegSetThreadName("LED update");

    Config *config = config_get_configuration();
    for(;;) {
        ControllerFault fault = controller_get_fault();
        if (fault == NO_FAULT)
        {
            if (packet_connect_event())
            {
                ws2812b_set_all(0x00FFFF);
                chThdSleepMilliseconds(100);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(100);
                ws2812b_set_all(0x00FFFF);
                chThdSleepMilliseconds(100);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(100);
                ws2812b_set_all(0x00FFFF);
                chThdSleepMilliseconds(100);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(100);
            }
            else if (controller_get_state() == RUNNING)
            {
                float command = controller_get_command_current();
                if (command > 0.0) // Forward
                {
                    float pcnt = command / config->maxCurrent;
                    ws2812b_set_all(0x00FF00);
                    chThdSleepMilliseconds((int)(pcnt * 400));
                    if (pcnt < 1.0)
                    {
                        ws2812b_set_all(0);
                        chThdSleepMilliseconds((int)((1 - pcnt) * 400));
                    }
                }
                else if (command < 0.0) // Reverse
                {
                    float pcnt = -command / config->maxCurrent;
                    ws2812b_set_all(0xFF0000);
                    chThdSleepMilliseconds((int)(pcnt * 400));
                    if (pcnt < 1.0)
                    {
                        ws2812b_set_all(0);
                        chThdSleepMilliseconds((int)((1 - pcnt) * 400));
                    }
                }
                else // Neutral
                {
                    ws2812b_set_all(0xFFFF00);
                    chThdSleepMilliseconds(350);
                    ws2812b_set_all(0);
                    chThdSleepMilliseconds(50);
                }
            }
            else if (comm_usb_serial_is_active())
            {
                ws2812b_set_all(0x0000FF);
                chThdSleepMilliseconds(250);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(250);
            }
            else
            {
                ws2812b_set_all(0x0000FF);
                chThdSleepMilliseconds(500);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(500);
            }
        }
        else
        {
            for (int i = 0; i < (int)fault; i++)
            {
                ws2812b_set_all(0xFFA500);
                chThdSleepMilliseconds(250);
                ws2812b_set_all(0);
                chThdSleepMilliseconds(250);
            }
            chThdSleepMilliseconds(250);
        }
    }
}


int main(void) {
    halInit();
    chSysInit();

    gpio_init();
    config_init();
    chThdSleepMilliseconds(250);

    ws2812b_init();
    encoder_init();
    scope_init();
    controller_init();
    chThdCreateStatic(led_update_wa, sizeof(led_update_wa), NORMALPRIO, led_update, NULL);
    comm_init();
    comm_usb_serial_init();

    for(;;)
    {
        /*controller_print();*/
        chThdSleepMilliseconds(1);
    }
}
