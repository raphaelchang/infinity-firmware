#ifndef _DATATYPES_H_
#define _DATATYPES_H_

#include "ch.h"

typedef enum
{
	STOPPED,
	RUNNING,
	ZEROING
} ControllerState;

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
	EncoderType encoderType;
	CommInterface commInterface;
	ZSMMode zsmMode;
	uint8_t polePairs;
	float encoderZero;
	float pwmFrequency;
} Config;

#endif