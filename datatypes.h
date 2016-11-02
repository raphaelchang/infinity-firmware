#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include "ch.h"

typedef enum
{
    STOPPED,
    RUNNING,
    TONE,
    INDUCTANCE_MEASURE
} ControllerState;

typedef enum
{
    NONE,
    CURRENT,
    DUTY_CYCLE,
    SPEED
} ControllerMode;

typedef enum
{
    NO_FAULT,
    UNDERVOLTAGE,
    OVERVOLTAGE,
    OVERCURRENT,
    TEMPERATURE
} ControllerFault;

typedef enum
{
    AS5045B, // SSI, 12-bit angular, 10-bit incremental
    AS5145B, // SSI, 12-bit angular, 12-bit incremental
    AS5x47P, // SPI, 14-bit angular, 12-bit incremental
    HALL,
    SENSORLESS
} EncoderType;

typedef enum
{
    CAN,
    I2C,
    UART,
    PPM,
    NUNCHUK,
    NRF,
    CUSTOM
} CommInterface;

typedef enum
{
    SINUSOIDAL,
    MIDPOINT_CLAMP,
    TOP_CLAMP,
    BOTTOM_CLAMP,
    TOP_BOTTOM_CLAMP
} ZSMMode;

typedef struct
{
    volatile uint8_t CANDeviceID;
    volatile EncoderType encoderType;
    volatile CommInterface commInterface;
    volatile ZSMMode zsmMode;
    volatile uint8_t polePairs;
    volatile float encoderZero;
    volatile bool encoderInverted;
    volatile float pwmFrequency;
    volatile float currentKp;
    volatile float currentKi;
    volatile float maxDuty;
    volatile float maxCurrent;
    volatile float pllKp;
    volatile float pllKi;
    volatile float speedKp;
    volatile float speedKi;
    volatile float speedKd;
    volatile float motorResistance;
    volatile float motorInductance;
    volatile float motorFluxLinkage;
    volatile float observerGain;
    volatile bool forwardCAN;
    volatile uint32_t CANStatusRate;
    volatile bool sendStatusCAN;
} Config;

typedef enum
{
    PACKET_CONNECT = 0x00,
    PACKET_CONSOLE = 0x01,
    PACKET_USB_OVERRIDE_START = 0x02,
    PACKET_USB_OVERRIDE_END = 0x03,
    PACKET_SET_DUTY_CYCLE = 0x04,
    PACKET_SET_CURRENT = 0x05,
    PACKET_GET_DATA = 0x06,
} PacketID;

#endif
