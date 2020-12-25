#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "comm_usb.h"
#include "hw_conf.h"

/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/*
 * Serial over USB Driver structure.
 */
SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
    USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).     */
            0x02,          /* bDeviceClass (CDC).              */
            0x00,          /* bDeviceSubClass.                 */
            0x00,          /* bDeviceProtocol.                 */
            0x40,          /* bMaxPacketSize.                  */
            0x0483,        /* idVendor (ST).                   */
            0x5740,        /* idProduct.                       */
            0x0200,        /* bcdDevice.                       */
            1,             /* iManufacturer.                   */
            2,             /* iProduct.                        */
            3,             /* iSerialNumber.                   */
            1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data,
    vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
    /* Configuration Descriptor.*/
    USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
            0x02,          /* bNumInterfaces.                  */
            0x01,          /* bConfigurationValue.             */
            0,             /* iConfiguration.                  */
            0xC0,          /* bmAttributes (self powered).     */
            50),           /* bMaxPower (100mA).               */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
            0x00,          /* bAlternateSetting.               */
            0x01,          /* bNumEndpoints.                   */
            0x02,          /* bInterfaceClass (Communications
                              Interface Class, CDC section
                              4.2).                            */
            0x02,          /* bInterfaceSubClass (Abstract
                              Control Model, CDC section 4.3).   */
            0x01,          /* bInterfaceProtocol (AT commands,
                              CDC section 4.4).                */
            0),            /* iInterface.                      */
    /* Header Functional Descriptor (CDC section 5.2.3).*/
    USB_DESC_BYTE         (5),            /* bLength.                         */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                             Functional Descriptor.           */
    USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                             Functional Descriptor).          */
    USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
    USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
    /* ACM Functional Descriptor.*/
    USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                             Control Management Descriptor).  */
    USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
    /* Union Functional Descriptor.*/
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                             Functional Descriptor).          */
    USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                             Class Interface).                */
    USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                             Interface).                      */
    /* Endpoint 2 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
            0x03,          /* bmAttributes (Interrupt).        */
            0x0008,        /* wMaxPacketSize.                  */
            0xFF),         /* bInterval.                       */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
            0x00,          /* bAlternateSetting.               */
            0x02,          /* bNumEndpoints.                   */
            0x0A,          /* bInterfaceClass (Data Class
                              Interface, CDC section 4.5).     */
            0x00,          /* bInterfaceSubClass (CDC section
                              4.6).                            */
            0x00,          /* bInterfaceProtocol (CDC section
                              4.7).                            */
            0x00),         /* iInterface.                      */
    /* Endpoint 3 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
            0x02,          /* bmAttributes (Bulk).             */
            0x0040,        /* wMaxPacketSize.                  */
            0x00),         /* bInterval.                       */
    /* Endpoint 1 Descriptor.*/
    USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
            0x02,          /* bmAttributes (Bulk).             */
            0x0040,        /* wMaxPacketSize.                  */
            0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
    sizeof vcom_configuration_descriptor_data,
    vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
    USB_DESC_BYTE(4),                     /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
    USB_DESC_BYTE(38),                    /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
    'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
    'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
    USB_DESC_BYTE(52),                    /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    'I', 0, 'n', 0, 'f', 0, 'i', 0, 'n', 0, 'i', 0, 't', 0, 'y', 0,
    ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0,
    ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0,
    't', 0
    /*'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,*/
    /*'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,*/
    /*'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,*/
    /*'o', 0, 'r', 0, 't', 0*/
};

/*
 * Serial Number string.
 * TODO: Replace with firmware version
 */
static const uint8_t vcom_string3[] = {
    USB_DESC_BYTE(8),                     /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
#ifdef INFINITY_4_0
    '0' + 4, 0,
    '.', 0,
    '0' + 0, 0
#elif defined INFINITY_4_1
    '0' + 4, 0,
    '.', 0,
    '0' + 1, 0
#endif
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
    {sizeof vcom_string0, vcom_string0},
    {sizeof vcom_string1, vcom_string1},
    {sizeof vcom_string2, vcom_string2},
    {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
        uint8_t dtype,
        uint8_t dindex,
        uint16_t lang) {

    (void)usbp;
    (void)lang;
    switch (dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &vcom_device_descriptor;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &vcom_configuration_descriptor;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4)
                return &vcom_strings[dindex];
    }
    return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    sduDataTransmitted,
    sduDataReceived,
    0x0040,
    0x0040,
    &ep1instate,
    &ep1outstate,
    2,
    NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    0x0010,
    0x0000,
    &ep2instate,
    NULL,
    1,
    NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
    extern SerialUSBDriver SDU1;

    switch (event) {
        case USB_EVENT_RESET:
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();

            /* Enables the endpoints specified into the configuration.
               Note, this callback is invoked from an ISR so I-Class functions
               must be used.*/
            usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
            usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

            /* Resetting the state of the CDC subsystem.*/
            sduConfigureHookI(&SDU1);

            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
            chSysLockFromISR();

            /* Disconnection event on suspend.*/
            sduDisconnectI(&SDU1);

            chSysUnlockFromISR();
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
    }
    return;
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

    (void)usbp;

    osalSysLockFromISR();
    sduSOFHookI(&SDU1);
    osalSysUnlockFromISR();
}

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    sduRequestsHook,
    sof_handler
};

/*
 * Serial over USB driver configuration.
 */
const SerialUSBConfig serusbcfg = {
    &USBD1,
    USBD1_DATA_REQUEST_EP,
    USBD1_DATA_AVAILABLE_EP,
    USBD1_INTERRUPT_REQUEST_EP
};

#define SERIAL_RX_BUFFER_SIZE		(2048)
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static THD_WORKING_AREA(serial_read_thread_wa, 512);
static THD_WORKING_AREA(serial_process_thread_wa, 4096);
static mutex_t send_mutex;
static thread_t *process_tp;

static THD_FUNCTION(serial_read_thread, arg) {
    (void)arg;

    chRegSetThreadName("USB serial read");

    uint8_t buffer[128];
    int i;
    int len;
    int had_data = 0;

    for(;;) {
        len = chSequentialStreamRead(&SDU1, (uint8_t*) buffer, 1);

        for (i = 0; i < len; i++) {
            serial_rx_buffer[serial_rx_write_pos++] = buffer[i];

            if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
                serial_rx_write_pos = 0;
            }

            had_data = 1;
        }

        if (had_data) {
            chEvtSignal(process_tp, (eventmask_t) 1);
            had_data = 0;
        }
        chThdSleepMilliseconds(1);
    }
}

static THD_FUNCTION(serial_process_thread, arg) {
    (void)arg;

    chRegSetThreadName("USB serial process");

    process_tp = chThdGetSelfX();

    for(;;) {
        chEvtWaitAny((eventmask_t) 1);

        while (serial_rx_read_pos != serial_rx_write_pos) {
            packet_process_byte(serial_rx_buffer[serial_rx_read_pos++]);

            if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
                serial_rx_read_pos = 0;
            }
        }
    }
}

void comm_usb_send(unsigned char *buffer, unsigned int len) {
    chMtxLock(&send_mutex);
    chSequentialStreamWrite(&SDU1, buffer, len);
    chMtxUnlock(&send_mutex);
}

void comm_usb_serial_init(void) {
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    chMtxObjectInit(&send_mutex);

    chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa), NORMALPRIO, serial_read_thread, NULL);
    chThdCreateStatic(serial_process_thread_wa, sizeof(serial_process_thread_wa), NORMALPRIO, serial_process_thread, NULL);
}

int comm_usb_serial_is_active(void) {
    return SDU1.config->usbp->state == USB_ACTIVE;
}
