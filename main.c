#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include "comm.h"
#include "led_rgb.h"
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
                led_rgb_set(0x00FFFF);
                chThdSleepMilliseconds(100);
                led_rgb_set(0);
                chThdSleepMilliseconds(100);
                led_rgb_set(0x00FFFF);
                chThdSleepMilliseconds(100);
                led_rgb_set(0);
                chThdSleepMilliseconds(100);
                led_rgb_set(0x00FFFF);
                chThdSleepMilliseconds(100);
                led_rgb_set(0);
                chThdSleepMilliseconds(100);
            }
            else if (controller_get_state() == RUNNING)
            {
                float command = controller_get_command_current();
                if (command > 0.0) // Forward
                {
                    float pcnt = command / config->maxCurrent;
                    led_rgb_set(0x00FF00);
                    if (pcnt >= 1.0 / 400)
                        chThdSleepMilliseconds((int)(pcnt * 400));
                    if (pcnt < 1.0)
                    {
                        led_rgb_set(0);
                        chThdSleepMilliseconds((int)((1 - pcnt) * 400));
                    }
                }
                else if (command < 0.0) // Reverse
                {
                    float pcnt = -command / config->maxCurrent;
                    led_rgb_set(0xFF0000);
                    if (pcnt >= 1.0 / 400)
                        chThdSleepMilliseconds((int)(pcnt * 400));
                    if (pcnt < 1.0)
                    {
                        led_rgb_set(0);
                        chThdSleepMilliseconds((int)((1 - pcnt) * 400));
                    }
                }
                else // Neutral
                {
                    led_rgb_set(0xFF5500);
                    chThdSleepMilliseconds(350);
                    led_rgb_set(0);
                    chThdSleepMilliseconds(50);
                }
            }
            else if (comm_usb_serial_is_active())
            {
                led_rgb_set(0x0000FF);
                chThdSleepMilliseconds(250);
                led_rgb_set(0);
                chThdSleepMilliseconds(250);
            }
            else
            {
                led_rgb_set(0x0000FF);
                chThdSleepMilliseconds(500);
                led_rgb_set(0);
                chThdSleepMilliseconds(500);
            }
        }
        else
        {
            for (int i = 0; i < (int)fault; i++)
            {
                led_rgb_set(0xFF1100);
                chThdSleepMilliseconds(250);
                led_rgb_set(0);
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

    led_rgb_init();
    encoder_init();
    scope_init();
    controller_init();
    chThdCreateStatic(led_update_wa, sizeof(led_update_wa), NORMALPRIO, led_update, NULL);
    comm_init();
    comm_usb_serial_init();

    for(;;)
    {
        /*controller_print();*/
        chThdSleepMilliseconds(10);
    }
}
