#ifndef _COMM_H_
#define _COMM_H_

#include "halconf.h"
#include "ch.h"
#include "hal.h"
#include "datatypes.h"

void comm_init(void);
void comm_set_usb_override(bool enable);
bool comm_get_usb_override(void);

#endif
