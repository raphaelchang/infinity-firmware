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

static THD_WORKING_AREA(led_update_wa, 1024);
static THD_FUNCTION(led_update, arg) {
    (void)arg;

    chRegSetThreadName("LED update");

    for(;;) {
        chThdSleepMilliseconds(10);
    }
}


int main(void) {
	halInit();
	chSysInit();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	chThdSleepMilliseconds(1000);

	comm_usb_serial_init();
	comm_init();
	ws2812b_init();
    encoder_init();
	controller_init();

    chThdCreateStatic(led_update_wa, sizeof(led_update_wa), NORMALPRIO, led_update, NULL);
	for(;;)
	{
		if (comm_usb_serial_is_active())
		{
			uint8_t test[20] = "Hello world";
			chprintf((BaseSequentialStream *)&SDU1, "%s\n", test);
		}
		chThdSleepMilliseconds(1000);
	}
}