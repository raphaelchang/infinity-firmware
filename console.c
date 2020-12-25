#include "console.h"
#include "packet.h"
#include "datatypes.h"
#include "ch.h"
#include "hal.h"
#include "comm_usb.h"
#include <string.h>
#include <stdio.h>
#include "comm.h"
#include "controller.h"
#include "config.h"
#include "scope.h"

void console_process_command(char *command)
{
    enum { kMaxArgs = 64 };
    int argc = 0;
    char *argv[kMaxArgs];

    char *p2 = strtok(command, " ");
    while (p2 && argc < kMaxArgs) {
        argv[argc++] = p2;
        p2 = strtok(0, " ");
    }
    if (argc == 0) {
        console_printf("No command received\n");
        console_printf("\r\n");
        return;
    }
    if (strcmp(argv[0], "ping") == 0) {
        console_printf("pong\n");
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "mem") == 0) {
        size_t n, size;
        n = chHeapStatus(NULL, &size);
        console_printf("core free memory : %u bytes\n", chCoreGetStatusX());
        console_printf("heap fragments   : %u\n", n);
        console_printf("heap free total  : %u bytes\n", size);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "threads") == 0) {
        thread_t *tp;
        static const char *states[] = {CH_STATE_NAMES};
        console_printf("    addr    stack prio refs     state           name time    \n");
        console_printf("-------------------------------------------------------------\n");
        tp = chRegFirstThread();
        do {
            console_printf("%.8lx %.8lx %4lu %4lu %9s %14s %lu\n",
                    (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                    (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                    states[tp->p_state], tp->p_name, (uint32_t)tp->p_time);
            tp = chRegNextThread(tp);
        } while (tp != NULL);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "uptime") == 0) {
        console_printf("System uptime: %d seconds\n", ST2S(chVTGetSystemTime()));
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "voltage") == 0) {
        float vbus = controller_get_bus_voltage();
        console_printf("Bus voltage: %.2f volts\n", (double)vbus);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "temp") == 0) {
        float temp = controller_get_temperature();
        console_printf("Board temperature: %.2f degrees C\n", (double)temp);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "usb_override_set") == 0) {
        comm_set_usb_override(true);
        console_printf("Enabling USB control\n");
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "usb_override_unset") == 0) {
        comm_set_usb_override(false);
        console_printf("Disabling USB control\n");
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "current_set") == 0) {
        if (argc == 2)
        {
            if (comm_get_usb_override())
            {
                float curr = 0.0;
                sscanf(argv[1], "%f", &curr);
                console_printf("Setting current to %.2f amps\n", (double)curr);
                controller_set_current(curr);
            }
            else
            {
                console_printf("USB control not enabled\n");
            }
        }
        else
        {
            console_printf("Usage: current_set [current]\n");
        }
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "speed_set") == 0) {
        if (argc == 2)
        {
            if (comm_get_usb_override())
            {
                float rpm = 0.0;
                sscanf(argv[1], "%f", &rpm);
                console_printf("Setting speed to %.2f RPM\n", (double)rpm);
                controller_set_speed(rpm);
            }
            else
            {
                console_printf("USB control not enabled\n");
            }
        }
        else
        {
            console_printf("Usage: speed_set [speed]\n");
        }
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "stop") == 0) {
        console_printf("Stopping motor\n");
        controller_disable();
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "zero") == 0) {
        console_printf("Finding encoder zero...\n");
        float zero;
        bool inverted;
        bool result = controller_encoder_zero(10.0, &zero, &inverted);
        if (result)
        {
            console_printf("Found zero: %.2f\n", (double)zero);
            if (inverted)
                console_printf("Encoder inverted\n");
            else
                console_printf("Encoder not inverted\n");
        }
        else
        {
            console_printf("Zeroing failed\n");
        }
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "res") == 0) {
        console_printf("Measuring winding resistance...\n");
        float res_tmp = 0.0;
        float i_last = 0.0;
        for (float i = 2.0; i < (30.0 / 2.0); i *= 1.5) {
            res_tmp = controller_measure_resistance(i, 20);

            if (i > (0.5 / res_tmp)) {
                i_last = i;
                break;
            }
        }

        if (i_last < 0.01) {
            i_last = (30.0 / 2.0);
        }

        float res = controller_measure_resistance(i_last, 500);
        console_printf("Resistance at %.2f amps: %.4f ohms\n", i_last, res);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "ind") == 0) {
	console_printf("Measuring winding inductance...\n");
	float duty_last = 0.03;
	/*for (float i = 0.02; i < 0.5; i *= 1.5) {*/
	    /*float i_tmp;*/
	    /*controller_measure_inductance(i, 20, &i_tmp);*/

	    /*if (i_tmp >= i_last) {*/
		/*duty_last = i;*/
		/*break;*/
	    /*}*/
	/*}*/
        float i_tmp = 0.0;
	float ind = controller_measure_inductance(duty_last, 500, &i_tmp);
	console_printf("Inductance with %.2f duty cycle at %.2f amps: %.4f microhenries\n", duty_last, i_tmp, ind);
	console_printf("\r\n");
    }
    else if (strcmp(argv[0], "angles") == 0) {
        console_printf("Actual angle: %.4f\n", controller_get_encoder_angle());
        console_printf("Observer angle: %.4f\n", controller_get_observer_angle());
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "faults") == 0) {
        console_printf("Faults: %.2f\n", controller_get_fault_value());
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "looptime") == 0) {
        console_printf("Last loop time: %.1f microseconds\n", controller_get_looptime() * 1e6);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "write") == 0) {
        console_printf("Writing configuration to EEPROM...\n");
        bool res = config_write();
        if (res)
            console_printf("Done.\n");
        else
            console_printf("Failed.\n");
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "read") == 0) {
        Config temp;
        config_read(&temp);
        console_printf("\r\n");
    }
    else if (strcmp(argv[0], "scope_dump") == 0) {
        if (argc == 2)
        {
            int channel;
            sscanf(argv[1], "%d", &channel);
            float buffer[256];
            uint16_t size = scope_get_data(channel, 256, buffer);
            for (uint16_t i = 0; i < size; i++)
            {
                console_printf("%f\n", buffer[i]);
            }
        }
        else
        {
            console_printf("Usage: scope_dump [channel]\n");
        }
        console_printf("\r\n");
    }
    else
    {
        console_printf("%s: command not found\n", argv[0]);
        // console_printf("type help for a list of available commands\n");
        console_printf("\r\n");
    }
}

void console_printf(char* format, ...) {
    va_list arg;
    va_start (arg, format);
    int len;
    static char print_buffer[255];

    print_buffer[0] = PACKET_CONSOLE;
    len = vsnprintf(print_buffer+1, 254, format, arg);
    va_end (arg);

    if(len > 0) {
        packet_send_packet((unsigned char*)print_buffer, (len<254)? len+1: 255);
    }
}
