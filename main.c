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

static THD_WORKING_AREA(led_update_wa, 1024);
static THD_FUNCTION(led_update, arg) {
    (void)arg;

    chRegSetThreadName("LED update");

    for(;;) {
    	ControllerFault fault = controller_get_fault();
    	if (fault == NO_FAULT)
    	{
			ws2812b_set_all(0x0000FF);
	        chThdSleepMilliseconds(500);
			ws2812b_set_all(0);
	        chThdSleepMilliseconds(500);
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
	
	comm_init();
	ws2812b_init();
    encoder_init();
	controller_init();
	comm_usb_serial_init();

    chThdCreateStatic(led_update_wa, sizeof(led_update_wa), NORMALPRIO, led_update, NULL);
	for(;;)
	{
		// controller_print();
		chThdSleepMilliseconds(1);
	}
}