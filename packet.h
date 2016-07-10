#ifndef PACKET_H_
#define PACKET_H_

#include "ch.h"

void packet_process_byte(uint8_t byte);
void packet_send_packet(unsigned char *data, unsigned int len);
bool packet_connect_event(void);

#endif /* PACKET_H_ */