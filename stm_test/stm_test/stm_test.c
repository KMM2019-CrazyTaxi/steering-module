/*
 * stm_test.c
 * Created: 2019-11-06 09:51:10
 *  Author: ahmha968
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <util/delay.h>



int main(void)
{	
	DDRD |= 0xFF;
	
	TCCR1A |=   (1<<WGM11) | (1<<WGM12) | (1<<WGM13) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=  (0<<WGM10) | (1<<WGM11) | (1<<WGM12) | (1<<WGM13);
	ICR1 = 19999 ;
	
	
		
	unsigned int NEUTRAL = ICR1 -1400; // 1.43ms
	unsigned int LEFT = ICR1 - 1210;
	unsigned int RIGHT= ICR1 -1650;
	
	EICRA |= 1<<ISC20 | 1<<ISC21;
	EIMSK |= 1<<INT2;
	EIFR |= 1<<INTF2;
	sei();               // Enable global interrupts
	
	OCR1A = NEUTRAL;
	OCR1B = NEUTRAL;
	
    while(1)
    {
			OCR1A = NEUTRAL;
			OCR1B = NEUTRAL;
		_delay_ms(3000);
				OCR1B = LEFT;
		_delay_ms(500);
						OCR1B = RIGHT;
		_delay_ms(100);
                
		
    }
}

ISR(INT2_vect)
{
	OCR1A -=10;
}