/*
 * SPI_dev.c
 *
 * Created: 2019-11-13 12:30:11
 *  Author: anglo547
 */ 


#include <avr/io.h>

#define F_CPU 1000000UL  // 1 MHz
#include <util/delay.h>

#include <avr/interrupt.h>

#define DDR_SPI DDRB
#define DD_MISO PORTB6


/* SPI Constants */
#define SPI_PROTOCOL_READ_BYTES 3
#define SPI_START 0xAA
#define SPI_NAN 0x00
#define SPI_FINISHED 0x66
#define SPI_RESTART 0x99
#define SPI_ERROR 0x55
#define SPI_ACK 0xEE

/* SPI hardcoded speed för PWM*/
#define SPI_SPEED_NEUTRAL 0x00
#define SPI_SPEED_LOW 0x10
#define SPI_SPEED_MIDDLE 0x20
#define SPI_BACK 0x30

#define SPI_ANGLE_NEUTRAL 0x00
#define SPI_ANGLE_LEFT 0x10
#define SPI_ANGLE_RIGHT 0x20


/* Global variables */
int8_t speed_cm = 0x00;
int8_t angle_cm = 0x11;

uint8_t check_byte = 0x22;

unsigned int NEUTRAL; // 1.43ms
unsigned int LEFT ;
unsigned int RIGHT;

unsigned int SPEED_LOW;
unsigned int SPEED_MIDDLE;
unsigned int SPEED_BACK; //1.38 ms


/* Prototypes */
uint8_t calc_check_byte(uint8_t *buffer, uint8_t size);

/* 
 * The steering module SPI interrupt routine.
 */
/*
ISR(SPI_STC_vect) {
	uint8_t spi_buffer[6];
	
	SPI_SlaveReceiveBytes(spi_buffer, PROTOCOL_READ_BYTES);
	
	check_byte = calc_check_byte(spi_buffer, PROTOCOL_READ_BYTES);
	
	speed_cm = (int8_t) spi_buffer[1];
	angle_cm = (int8_t) spi_buffer[2];
	
}
*/

uint8_t calc_check_byte(uint8_t *buffer, uint8_t size) {
	uint8_t check = buffer[0];
	for (uint8_t i = 1; i < size; i++) {
		check ^= buffer[i];
	}

	return check;
}

void init(void) {
	DDRA = 0xFF;
}

void SPI_SlaveInit(void)
{
	/* Set MISO output, all others input */
	DDR_SPI = (1<<DD_MISO);
	/* Enable SPI */
	SPCR = (1<<SPE);
}

uint8_t SPI_SlaveReceive(void)
{
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)))
	;
	/* Return Data Register */
	return SPDR;
}

uint8_t SPI_tranceive(uint8_t trans_data) {
	SPDR = trans_data;
	/* Wait for reception complete */
	while(!(SPSR & (1<<SPIF)));
	/* Return Data Register */
	return SPDR;
}


void SPI_SlaveReceiveBytes(uint8_t *buffer, uint8_t size) {
	for (int i = 0; i < size; i++) {
		buffer[i] = SPI_SlaveReceive();
	}
}


void count_port_a(void) {
	while (1) {
		for (uint8_t a = 0; a < 0xFF; ++a) {
			PORTA = a;
			
			for (uint8_t t = 1; t != 0; ++t){
				for (uint8_t i = 1; i != 0; ++i){}
			}
			
		}

	}
}


void print_buffer_port_a(uint8_t *buffer, uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		PORTA = buffer[i];
		_delay_ms(1000);
	}
}


/*
 * Perform communication with the central module in accordance to the protocol. 
 * Returns 1 if the communication was successful or 0 if it failed.
 * Ideally we want as few function calls as possible in this method.
 */
void read_data_send_check(void) {
	/* Read Speed byte */
	speed_cm = SPI_tranceive(SPI_NAN);
	
	PORTA = 0xEE;
	
	/* Read Angle byte */
	angle_cm = SPI_tranceive(SPI_NAN);

	/* Send CHECK byte */
	check_byte = speed_cm ^ angle_cm;
	SPI_tranceive(check_byte);	
}


void PWM_init() {
	DDRD |= 0xFF;
	
	TCCR1A |=  (1<<WGM11) | (1<<WGM12) | (1<<WGM13) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=  (0<<WGM10) | (1<<WGM11) | (1<<WGM12) | (1<<WGM13);
	ICR1 = 19999 ;
	
	
	NEUTRAL = ICR1 -1400; // 1.43ms
	LEFT = ICR1 - 1210;
	RIGHT= ICR1 -1650;

	SPEED_LOW = ICR1 - 1460;
	SPEED_MIDDLE = ICR1 - 1475;
	SPEED_BACK = ICR1 - 1350; //1.38 ms
	
}


void impl_speed_direction(void) {
	
	if (speed_cm == SPI_SPEED_NEUTRAL) {
		OCR1A = NEUTRAL;
	} else if (speed_cm == SPI_SPEED_LOW) {
		OCR1A = SPEED_LOW;
	} else if (speed_cm == SPI_SPEED_MIDDLE) {
		OCR1A = SPEED_MIDDLE;
	}else if (speed_cm == SPI_BACK) {
		OCR1A = SPEED_BACK;
	}else {
		PORTA = 0xAE;
	}
	
	if (angle_cm == SPI_ANGLE_NEUTRAL) {
		OCR1B = NEUTRAL;
	} else if (angle_cm ==  SPI_ANGLE_LEFT) {
		OCR1B = LEFT;
	} else if (angle_cm ==  SPI_ANGLE_RIGHT) {
		OCR1B = RIGHT;
	} else {
		PORTA = 0xEA;
	}

}


void speed_controller(signed char  speed) {
		// speeds choses between 0-127 or reversed speeds between 0-(-127),
		// OCR1A = 18600(the compare value which is used with the counter ICR1 to create a fast PWM) give neutral speed.
		// The highest speed will be reached when the OCR1A = 18219 (when direction = 127) which gives a PWM signal = (2ms high signal from 20 ms).
		// for example when speed = 0 so OCR1A = 18600 which will give neutral speed.
		OCR1A = 18600 - (speed * 3);
		
	}
	
	
	
void direction_controller(signed char direction ) {
		// Right directions between 0-127 and left directions between 0-(-127), OCR1B = 18600(the compare value with the counter ICR1) give neutral direction.
		// The far right direction will be near to OCR1B = 18219 (when direction = 127) which gives a PWM signal  = (2ms high signal from 20ms),
		// while the far left will be near to OCR1B = 18981 (when direction = -127) which gives a PWM signal  = (1ms high signal from 20ms).
		OCR1B = 18600 - (direction * 3);
		
	}



int main(void) {


	SPI_SlaveInit();
	PWM_init();
	init();

	PORTA = 0xBB; 	
	
	uint8_t spi_rdy = 0;
	uint8_t spi_success = 0;
	
	uint8_t spi_read = 0;
	uint8_t cntr = 0;
	

	
	OCR1A = SPEED_BACK;
	OCR1B = NEUTRAL;
	
	while(1) {
		
		spi_rdy = 0;
		
		/* SPI wait for start byte from central module */
		while (spi_rdy == 0) {
			spi_read = SPI_tranceive(SPI_ACK);
			spi_rdy = (spi_read == SPI_START);
			
			cntr++;
		}
		cntr = 0;
		
		read_data_send_check();
		
		/* Check if communication was a success */
		spi_success = SPI_tranceive(SPI_NAN) == SPI_FINISHED;
		
		if (spi_success) {
			//impl_speed_direction();
			speed_controller(speed_cm);
			direction_controller(angle_cm);
			
		}
	}
}

