#include "packet.h"
#include "datatypes.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"
#include <string.h>
#include <stdio.h>
#include "console.h"
#include "comm.h"
#include "controller.h"

static uint16_t packet_index;
static uint8_t packet_buffer[2048];
static bool in_packet = false;
static bool connect_event = false;

static void process_packet(unsigned char *data, unsigned int len);
static int16_t parse_int16(const uint8_t *buffer, int32_t index);
static uint16_t parse_uint16(const uint8_t *buffer, int32_t index);
static int32_t parse_int32(const uint8_t *buffer, int32_t index);
static uint32_t parse_uint32(const uint8_t *buffer, int32_t index);

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
        case PACKET_CONNECT:
            connect_event = true;
            break;
        case PACKET_CONSOLE:
            data[len] = '\0';
            console_process_command(data);
            break;
        case PACKET_USB_OVERRIDE_START:
            comm_set_usb_override(true);
            break;
        case PACKET_USB_OVERRIDE_END:
            comm_set_usb_override(false);
            break;
        case PACKET_SET_DUTY_CYCLE:
            if (comm_get_usb_override())
            {
                controller_set_duty(parse_int16(data, 0) / 10000.0);
            }
            break;
        case PACKET_SET_CURRENT:
            if (comm_get_usb_override())
            {
                controller_set_current(parse_int16(data, 0) / 1000.0);
            }
            break;
        default:
            break;
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

bool packet_connect_event(void)
{
    if (connect_event)
    {
        connect_event = false;
        return true;
    }
    return false;
}

static int16_t parse_int16(const uint8_t *buffer, int32_t index) {
    int16_t res =   ((uint16_t) buffer[index]) << 8 |
        ((uint16_t) buffer[index + 1]);
    index += 2;
    return res;
}

static uint16_t parse_uint16(const uint8_t *buffer, int32_t index) {
    uint16_t res =  ((uint16_t) buffer[index]) << 8 |
        ((uint16_t) buffer[index + 1]);
    index += 2;
    return res;
}

static int32_t parse_int32(const uint8_t *buffer, int32_t index) {
    int32_t res =   ((uint32_t) buffer[index]) << 24 |
        ((uint32_t) buffer[index + 1]) << 16 |
        ((uint32_t) buffer[index + 2]) << 8 |
        ((uint32_t) buffer[index + 3]);
    index += 4;
    return res;
}

static uint32_t parse_uint32(const uint8_t *buffer, int32_t index) {
    uint32_t res =  ((uint32_t) buffer[index]) << 24 |
        ((uint32_t) buffer[index + 1]) << 16 |
        ((uint32_t) buffer[index + 2]) << 8 |
        ((uint32_t) buffer[index + 3]);
    index += 4;
    return res;
}
