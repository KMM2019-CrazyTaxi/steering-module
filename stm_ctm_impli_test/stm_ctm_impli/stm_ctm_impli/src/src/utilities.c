/*
 * utilities.c
 *
 * Created: 2019-11-10 13:28:23
 *  Author: herap603
 */ 

#include "utilities.h"

void utilities_busy_wait_ms(const uint8_t ms)
{
	for (uint8_t ms_waited = 0; ms_waited < ms; ++ms_waited) 
	{
		for (uint8_t i = 0; i < 2; ++i)
		{
			for (uint8_t j = 0; j < 121; ++j)
			{
				__asm__ __volatile__ ("nop");
			}
		}
	}
}

void utilities_busy_wait_s(const uint8_t s) {
	for (uint8_t i = 0; i < s; ++i)
	{
		for (uint8_t j = 0; j < 4; ++j)
		{
			utilities_busy_wait_ms(250);	
		}
	}
}

void utilities_debug_output(const uint8_t* data, const uint8_t n_bytes)
{
	for (uint8_t i = 0; i < n_bytes; ++i)
	{
		PORTA = *(data + i);
		utilities_busy_wait_ms(1);
	}
}

void utilities_error(const uint8_t error_code)
{
	PORTA = error_code;
	while (1);
}
