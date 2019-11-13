/*
 * basic_io_test.c
 *
 * Created: 2019-11-05 10:29:16
 *  Author: herap603
 */ 

#include "basic_functionality_tests.h"

void run_regular_pulse_test(const uint8_t count_to)
{
	while (1) {
		for (uint8_t pulse = 0; pulse < count_to; ++pulse) {
			PORTA = pulse;
			for (uint8_t delay_cnt = 1; delay_cnt != 0; ++delay_cnt) { }
		}
	}
}
