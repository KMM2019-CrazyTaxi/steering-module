/*
 * utilities.h
 *
 * Created: 2019-11-10 13:28:08
 *  Author: herap603
 */ 


#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <stdint.h>
#include <asf.h>

/*
 * Busy waits for a given number of ms.
 *
 * ms: The number of ms to wait.
 */
void utilities_busy_wait_ms(const uint8_t ms);

/*
 * Busy waits for a given number of s.
 *
 * s: The number of s to wait.
 */
void utilities_busy_wait_s(const uint8_t s);

/*
 * Outputs several bytes of data to port A with one ms between each byte.
 *
 * data: A pointer to the first byte of data to output.
 * n_bytes: The number of bytes to output.
 *
 * Assumes: PORTA has been configured as output.
 */
void utilities_debug_output(const uint8_t* data, const uint8_t n_bytes);

/*
 * Outputs an error code to port A and blocks indefinitely.
 *
 * error_code: The code to output on port A.
 *
 * Assumes: PORTA has been configured as output.
 */
void utilities_error(const uint8_t error_code);

#endif /* UTILITIES_H_ */