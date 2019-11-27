/*
 * stm_test.c
 * Created: 2019-11-06 09:51:10
 *  Author: ahmha968
 */ 


#define F_CPU 1000000UL
#include <avr/io.h>	
#include <avr/interrupt.h>
#include <util/delay.h>


unsigned int NEUTRAL; 
unsigned int NEUTRAL_direction;
unsigned int acceleration_rate_speed ;
unsigned int acceleration_rate_direction;

void speed_controller(signed char  speed) {
	// speeds choses between 0-127 or reversed speeds between 0-(-127),
	// OCR1A = 18509(the compare value which is used with the counter ICR1 to create a fast PWM) give neutral speed.
	// The highest speed will be reached when the OCR1A = 18219 (when direction = 127) which gives a PWM signal = (2ms high signal from 20 ms).
	// for example when speed = 0 so OCR1A = 18600 which will give neutral speed.
	
	OCR1A = NEUTRAL - (speed * acceleration_rate_speed);
	
}



void direction_controller(signed char direction ) {
	// Right directions between 0-127 and left directions between 0-(-127),
	// OCR1B = 18509(the compare value with the counter ICR1 : counter_limit_const counter_limit_const) give neutral direction.
	// The far right direction will be near to OCR1B = 18219 (when direction = 127) which gives a PWM signal  = (2ms high signal from 20ms),
	// while the far left will be near to OCR1B = 18981 (when direction = -127) which gives a PWM signal  = (1ms high signal from 20ms).

	OCR1B = NEUTRAL_direction - (direction * acceleration_rate_direction);
	
}

int main(void)
{	
	DDRD |= 0xFF;
	TCCR1A |=   (1<<WGM11) | (1<<COM1B0 |1<<COM1B1) | (1<<COM1A0 |1<<COM1A1);
	TCCR1B |=   (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	ICR1 = F_CPU/50; //19999 ; // 50 hz needed for the motor and controlling servo.
	
	
	 NEUTRAL = ICR1-1490 ; // pulse width 1.5ms
	 acceleration_rate_speed = 10;
	 acceleration_rate_direction = 4;
	 NEUTRAL_direction = ICR1-1430; // pulse width 1,43
		
	
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
	
	
	//OCR1B = NEUTRAL; // 
	OCR1B = NEUTRAL_direction; // 
	OCR1A = NEUTRAL;  // 
	
    while(1)
    {


	
		
		
		
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
