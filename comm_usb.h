#ifndef COMM_USB_SERIAL_H_
#define COMM_USB_SERIAL_H_
#include "chprintf.h"

#define USB_PRINT(format, ...) if (comm_usb_serial_is_active()) { chprintf((BaseSequentialStream *)&SDU1, format, __VA_ARGS__); }

extern SerialUSBDriver SDU1;

void comm_usb_serial_init(void);
int comm_usb_serial_is_active(void);

#endif /* COMM_USB_SERIAL_H_ */