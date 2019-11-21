/*
 * stm_test.c
 * Created: 2019-11-06 09:51:10
 *  Author: ahmha968
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <util/delay.h>


void speed_controller(signed char  speed) {
	// speeds choses between 0-127 or reversed speeds between 0-(-127),
	// OCR1A = 18600(the compare value which is used with the counter ICR1 to create a fast PWM) give neutral speed.
	// The highest speed will be reached when the OCR1A = 18219 (when direction = 127) which gives a PWM signal = (2ms high signal from 20 ms).
	// for example when speed = 0 so OCR1A = 18600 which will give neutral speed.
	
	unsigned int counter_limit_const = 18600;
	unsigned int acceleration_rate = 4;
	OCR1A = counter_limit_const - (speed * acceleration_rate );
	
}



void direction_controller(signed char direction ) {
	// Right directions between 0-127 and left directions between 0-(-127), OCR1B = 18600(the compare value with the counter ICR1) give neutral direction.
	// The far right direction will be near to OCR1B = 18219 (when direction = 127) which gives a PWM signal  = (2ms high signal from 20ms),
	// while the far left will be near to OCR1B = 18981 (when direction = -127) which gives a PWM signal  = (1ms high signal from 20ms).
	unsigned int counter_limit_const = 18600;
	unsigned int acceleration_rate = 3;
	OCR1B = counter_limit_const - (direction * acceleration_rate);
	
}

int main(void)
{	
	DDRD |= 0xFF;
	TCCR1A |=   (1<<WGM11) | (1<<WGM12) | (1<<WGM13) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=  (0<<WGM10) | (1<<WGM11) | (1<<WGM12) | (1<<WGM13);
	ICR1 = 19999 ; // 20 ms period needed for the motor and controlling servo.

		
	unsigned int NEUTRAL = ICR1 -1447; // 1.47ms
	unsigned int LEFT = ICR1 - 1210;
	unsigned int RIGHT= ICR1 -1650;
	
    // positive value for speed.
	
	signed char p = 127; // top forward speed.
	signed char o = 100; 
	signed char n = 90;
	signed char m = 80;
	signed char l = 70;
	signed char k = 50;
	signed char j = 40;
	signed char h = 30;
	signed char g = 20;
	signed char f = 14;
	signed char e = 11;
	signed char d = 10;
	signed char c = 3;
	signed char b = 2;
	signed char a = 1;
	signed char z = 0; // No movement.

	
	
	// negative value for reverse speed.
	
	signed char pp = -127; // top reverse speed. 
	signed char oo = -100;
	signed char nn = -90;
	signed char mm = -80;
	signed char ll = -70;
	signed char kk = -50;
	signed char jj = -40;
	signed char hh = -30;
	signed char gg = -20;
	signed char ff = -14;
	signed char ee = -11;
	signed char dd = -10;
	signed char cc = -3;
	signed char bb = -2;
	signed char aa = -1; 
	//signed char z = 0 ; // no movement.
	
	EICRA |= 1<<ISC20 | 1<<ISC21;
	EIMSK |= 1<<INT2;
	EIFR |= 1<<INTF2;
	sei();               // Enable global interrupts.
	
	//OCR1B = NEUTRAL; // 
	OCR1B = NEUTRAL; // 
	OCR1A = NEUTRAL;  // 
	
    while(1)
    {
	speed_controller(z);
		_delay_ms (1000);
		speed_controller(a);
		_delay_ms (1000);	
		speed_controller(b);
		_delay_ms (1000);	
		speed_controller(c);
		_delay_ms (1000);	
		speed_controller(d);
		_delay_ms (1000);	
		speed_controller(e);
		_delay_ms (1000);	
		speed_controller(f);
		_delay_ms (1000);	
		speed_controller(g);
		_delay_ms (1000);	
		speed_controller(h);
		_delay_ms (1000);	
		speed_controller(j);
		_delay_ms (1000);	
		speed_controller(k);
		_delay_ms (1000);	
		speed_controller(l);
		_delay_ms (1000);	
		speed_controller(m);
		_delay_ms (1000);	
		speed_controller(n);
		_delay_ms (1000);	
		speed_controller(o);
		_delay_ms (1000);	
		
		
		
		speed_controller(z);
		_delay_ms (1000);	
		speed_controller(aa);
		_delay_ms (1000);	
		speed_controller(bb);
		_delay_ms (1000);	
		speed_controller(cc);
		_delay_ms (1000);	
		speed_controller(dd);
		_delay_ms (1000);	
		speed_controller(ee);
		_delay_ms (1000);	
		speed_controller(ff);
		_delay_ms (1000);	
		speed_controller(gg);
		_delay_ms (1000);	
		speed_controller(hh);
		_delay_ms (1000);	
		speed_controller(jj);
		_delay_ms (1000);	
		speed_controller(kk);
		_delay_ms (1000);	
		speed_controller(ll);
		_delay_ms (1000);	
		speed_controller(mm);
		_delay_ms (1000);	
		speed_controller(nn);
		_delay_ms (1000);	
		speed_controller(oo);
		_delay_ms (1000);
		
    }
	return 0;
}


ISR(INT2_vect)
{
	OCR1A -=10;
}
