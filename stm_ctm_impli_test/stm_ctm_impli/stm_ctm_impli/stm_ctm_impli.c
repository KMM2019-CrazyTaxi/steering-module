/*
 * stm_ctm_impli.c
 *
 * Created: 2019-11-11 11:30:12
 *  Author: ahmha968
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


int main(void)
{
	DDRD |= 0xFF; // Make all the PD pins as output.
		
    //PWM fas mode configuration. 
	TCCR1A |=   (1<<WGM11) | (1<<WGM12) | (1<<WGM13) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=  (0<<WGM10) | (1<<WGM11) | (1<<WGM12) | (1<<WGM13);
	ICR1 = 19999 ;

	//Speed and directions constants.  
	unsigned int NEUTRAL = ICR1 -1400; // 1.43ms
	unsigned int LEFT = ICR1 - 1210;
	unsigned int RIGHT= ICR1 -1650;
	
	// speed variable. 
	OCR1A = NEUTRAL;
	// directions variable. 
	OCR1B = NEUTRAL;

	// The main loop.

	// initialize SPI slave device.
	void spi_init_slave(void)
	
	{
		// Interrupt controlled slave mode.
		// an SPI interrupt handler will be triggered to read data from  master and after reading 
		// the data the slave will trigger a global interrupt to handle other stuff.
		// in this mode the slave will be always listing to the master and 
		// stop doing any thing if the master need to tell something.  
		
		//DDRB = (0<<PB5);			// MOSI as input.
		DDRB = (1<<PB6);			// MISO as output.
		//DDRB = (1<<PB7);			// SCK  as input.
		SPCR = (1<<SPE);		// enable SPI.
		SPCR = (1<<SPIE);		// activate interrupts to get apply master order when transmission is complete.
		SPSR = (1<<SPI2X);		//  (0 <<SPR01)  | (0 <<SPR00); // relationship between SCK and oscillator clock frequency fosc/2.

	}

	// function to send data from master.
	unsigned char spi_tranceiver(unsigned char data)
	{
	
		SPDR = data;

		// wait until data are completely sent to the slave.
		while (!(SPSR & (1<<SPIF)))
			{
				return (SPDR);
			}
	}


	char SPI_SlaveReceive(void)
	{
			while (!(SPSR & (1<<SPIF)))
			{
				return (SPDR);
			}

	}

    


	sei(); //  enable global interrupt.
	OCR1A = NEUTRAL;// speed inti.
	OCR1B = NEUTRAL;// speed inti.
	// main loop
	while(1)
	{
		// in_data = SPI_SlaveReceive();
		//TODO:: Please write your application code
	}
	return 0;
}


//.
 ISR(SPI_STC_vect){

		// ode to execute
		// whenever transmission/reception 
		// is complete 
		unsigned int b = SPDR;
		OCR1A += b;

}
