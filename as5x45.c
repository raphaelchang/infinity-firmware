#include "encoder.h"

#include "ch.h"
#include "hal.h"

void as5x45_init(EncoderType type)
{

}

double as5x45_get_angle(void)
{

}

uint16_t as5x45_get_raw_position(void)
{
	int clock_p;
	unsigned int data = 0 ;
	digitalWrite (cs, LOW) ;
	for (byte i = 0 ; i < 12 ; i++)
	{
		digitalWrite (clk, LOW) ;
		for (int i = 0; i < 4; i++)
			__asm__("nop\n\t");
		digitalWrite (clk, HIGH) ;
		for (int i = 0; i < 10; i++)
			__asm__("nop\n\t");
		data = (data << 1) | digitalRead (dio) ;
	}
	byte status = 0;
	for (byte i = 0 ; i < 6 ; i++)
	{
		digitalWrite (clk, LOW) ;
		for (int i = 0; i < 4; i++)
			__asm__("nop\n\t");
		digitalWrite (clk, HIGH);
	for (int i = 0; i < 10; i++)
		__asm__("nop\n\t");
		status = (status << 1) | digitalRead (dio) ;
	}
	this->status = status >> 1;
	digitalWrite (cs, HIGH);
	return (4096 - data) % 4096;
}