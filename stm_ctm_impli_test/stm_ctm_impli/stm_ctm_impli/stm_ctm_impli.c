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
#include <avr/interrupt.h>
#include <util/delay.h>


	
	
void SPI_SlaveInit(void)
{

DDRB = 0b00000010;

SPCR = (1<<SPE);
}

char SPI_SlaveReceive(void)
{

while(!(SPSR & (1<<SPIF)))
;return SPDR;

}



int main(void)
{
	SPI_SlaveInit();
	// main loop
	DDRA = 0b11111111;
	while(1)
	{
		unsigned char temp = SPI_SlaveReceive();
        PORTA = temp;
		
	}
	return 0;
}
