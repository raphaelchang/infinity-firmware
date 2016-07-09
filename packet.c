#include "packet.h"
#include "datatypes.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include <string.h>
#include <stdio.h>

static uint16_t packet_index;
static uint8_t packet_buffer[2048];
static bool in_packet = false;

static void process_packet(unsigned char *data, unsigned int len);
static void packet_printf(char* format, ...);

void packet_process_byte(uint8_t byte)
{
	if (in_packet)
	{
		if (byte == '\n')
		{
			in_packet = false;
			process_packet(packet_buffer, packet_index);
			packet_index = 0;
		}
		else if (packet_index < 2048)
		{
			packet_buffer[packet_index++] = byte;
		}
	}
	else if (byte == 'P')
	{
		in_packet = true;
		packet_index = 0;
	}
}

static void process_packet(unsigned char *data, unsigned int len)
{
	uint8_t id = data[0];
	data++;
	len--;
	switch(id)
	{
		case PACKET_CONSOLE:
			data[len] = '\0';
			enum { kMaxArgs = 64 };
			int argc = 0;
			char *argv[kMaxArgs];

			char *p2 = strtok(data, " ");
			while (p2 && argc < kMaxArgs) {
				argv[argc++] = p2;
				p2 = strtok(0, " ");
			}
			if (strcmp(argv[0], "ping") == 0) {
				packet_printf("pong\n");
				packet_printf("\r\n");
			}
			else
			{
				packet_printf("%s: command not found\n", argv[0]);
				packet_printf("\r\n");
			}
			break;
		default:
			break;
	}
}

static void packet_printf(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = PACKET_CONSOLE + '0';
	len = vsnprintf(print_buffer+1, 254, format, arg);
	va_end (arg);

	if(len > 0) {
		packet_send_packet((unsigned char*)print_buffer, (len<254)? len+1: 255);
	}
}

void packet_send_packet(unsigned char *data, unsigned int len)
{
	uint8_t buffer[2048];
	uint8_t inx = 0;
	buffer[inx++] = 'P';
	memcpy(buffer + inx, data, len);
	inx += len;
	comm_usb_send(buffer, inx);
}