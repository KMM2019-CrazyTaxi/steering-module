/*
 * stm_test.c
 * Created: 2019-11-06 09:51:10
 *  Author: ahmha968
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>


#define F_CPU = 16000000UL


int main(void)
{	
	DDRD |= 0xFF;
	DDRB |= 0x00;
	
	TCCR1A |= 0<<WGM10 | 1<<WGM11  | 1<<COM1A1 | 1<<COM1B1  ;
	TCCR1B |= 1<<WGM12 | 1<<WGM13 | 1<<CS10 ;
	ICR1 = 19999;
	
	unsigned int NEUTRAL = ICR1 - 190;
	unsigned int FULL_REVERSE = ICR1 - 125;
	unsigned int FULL_FORWARD = ICR1 - 250;
	
	EICRA |= 1<<ISC20 | 1<<ISC21;
	EIMSK |= 1<<INT2;
	EIFR |= 1<<INTF2;
	sei();               // Enable global interrupts
	
	OCR1A = NEUTRAL;
	//OCR1B = NEUTRAL;
	
    while(1)
    {
    }
}

ISR(INT2_vect)
{
	OCR1A -= 1;
}