/*
 * SPI_dev.c
 *
 * Created: 2019-11-13 12:30:11
 *  Author: anglo547
 */ 


#include <avr/io.h>
#include <stdlib.h>

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

unsigned int NEUTRAL_SPEED; 
unsigned int NEUTRAL_DIRECTION; 

unsigned int counter_limit_const ;
float acceleration_rate_speed ;
unsigned int acceleration_rate_direction;

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


void PWM_init(void) {
	DDRD |= 0xFF;
	TCCR1A |=   (1<<WGM11) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=   (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	ICR1 = F_CPU/50; //19999 ; // 50 hz needed for the motor and controlling servo.

	NEUTRAL_SPEED = ICR1-1490 ; // pulse width 1.5ms
	acceleration_rate_speed = 2;
	acceleration_rate_direction = 4;
	NEUTRAL_DIRECTION = ICR1-1430; // pulse width 1,43 
}

/*
 * This function initiates timer3, which is used to track how long time has passed since
 * the last exchange with the central module. If too long time has passed, the overflow
 * interrupt from this timer will shut down the engine, preventing crashes if the connection
 * is lost.
 */
void initiate_abort_counter(void)
{
	PRR0 = PRR0 & ~(1 << PRTIM3);	// Enable COUNT3 circuit.
	TCCR3A = 0;	// Normal mode.
	TCCR3B = (0 << CS32) | (1 << CS31) | (0 << CS10);	// Prescaler 8 on 1 MHz system clock.
	TIMSK3 = (1 << TOIE3);	// Enable overflow interrupt
}


// speeds choses between 0-127 or reversed speeds between 0-(-127),
// OCR1A = 18509(the compare value which is used with the counter ICR1 to create a fast PWM) give neutral speed.
// The highest speed will be reached when the OCR1A = 18219 (when direction = 127) which gives a PWM signal = (2ms high signal from 20 ms).
// for example when speed = 0 so OCR1A = 18600 which will give neutral speed.
void speed_controller(signed char speed) {

		if (speed > 30) {
			speed = 30;
		} else if (speed < -30) {
			speed = -30;
		}
		if (speed > 0) {
			if (speed < 108) {
				speed += 20;
				} else {
				speed = 127;
			}
		} else if (speed < 0) {
			if (speed > -109) {
				speed -= 20;
			} else {
				speed = -128;
			}
		}
		
		OCR1A = NEUTRAL_SPEED - (unsigned int)(speed * acceleration_rate_speed);
		
	}
	
	
// Right directions between 0-127 and left directions between 0-(-127),
// OCR1B = 18509(the compare value with the counter ICR1 : counter_limit_const counter_limit_const) give neutral direction.
// The far right direction will be near to OCR1B = 18219 (when direction = 127) which gives a PWM signal  = (2ms high signal from 20ms),
// while the far left will be near to OCR1B = 18981 (when direction = -127) which gives a PWM signal  = (1ms high signal from 20ms).	
void direction_controller(signed char direction ) {
	
		OCR1B = NEUTRAL_DIRECTION - (direction * acceleration_rate_direction);
		
}
	

// Lost connection with central module, see comment on initiate_abort_counter
ISR(TIMER3_OVF_vect)
{
	speed_controller(0);
	direction_controller(0);
	PORTA = 0xF1;
}


ISR(INT2_vect)
{
	
	//Stop button.
	speed_controller(0);
	direction_controller(0);
	while(1){
		
	}
}

int main(void) {

	// Stop button configurations.
	EICRA |= 1<<ISC20 | 1<<ISC21;
	EIMSK |= 1<<INT2;
	EIFR |= 1<<INTF2;
	SPI_SlaveInit();
	PWM_init();
	init();
	initiate_abort_counter();
	sei();               // Enable global interrupts.

	PORTA = 0xBB; 	
	
	uint8_t spi_rdy = 0;
	uint8_t spi_success = 0;

	uint8_t spi_read = 0;
	
	speed_controller(0);
	direction_controller(0);
	
	while(1) {
		PORTA = 0xFF;
		
		
		spi_rdy = 0;
		
		// SPI wait for start byte from central module
		while (spi_rdy == 0) {
			spi_read = SPI_tranceive(SPI_ACK);
			spi_rdy = (spi_read == SPI_START);
			
		}

		read_data_send_check();
		
		// Check if communication was a success
		spi_success = SPI_tranceive(SPI_NAN) == SPI_FINISHED;
		
		if (spi_success) {
			// Update control values
			speed_controller(speed_cm);
			direction_controller(angle_cm);
			
		}
		
	}
}

