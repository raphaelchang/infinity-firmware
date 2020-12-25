#ifndef _COMM_CAN_H_
#define _COMM_CAN_H_

#include "ch.h"
#include "hal.h"
#include "can_data.h"

#define CAN_BROADCAST 0xFF

void comm_can_init(void);
void comm_can_update(void);
void comm_can_forward_commands(void);
void comm_can_transmit(uint8_t receiver, CANPacketID packetID, uint8_t *data, uint8_t len);

#endif
