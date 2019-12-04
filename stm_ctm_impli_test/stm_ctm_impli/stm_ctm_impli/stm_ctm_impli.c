/*
 * stm_ctm_impli.c
 *
 * Created: 2019-11-11 11:30:12
 *  Author: ahmha968
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>


int main(void)
{
	DDRD = 0b11111111;
	PORTD =0b11111111;
	while(1)
	{

	}
	return 0;
}
