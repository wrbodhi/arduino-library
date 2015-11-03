/*
 * Copyright (c) 2015 Wind River Systems, Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at 
 * 
 * http://www.apache.org/licenses/LICENSE-2.0 
 * 
 * Unless required by applicable law or agreed to in writing, software distributed 
 * under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES 
 * OR CONDITIONS OF ANY KIND, either express or implied. See the License for
 * the specific language governing permissions and limitations under the License. 
 */

#include "arduino.h"

#include <zephyr.h>
#include <gpio.h>
#include <i2c.h>
#include <pwm.h>


#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

extern void hardware_init();
extern void arduino (void);
extern void main (void);
extern void setup (void);
extern void loop (void);

/*******************************************************************************
* arduino - Entry point for simple Arduino API
*
* Initializes the hardware then calls main (where Helix App Cloud Debugger stops).
*
* Main references users setup and loop routines.
*
* Returns:  void
*/
void arduino (void)
{
	hardware_init();
	main();
}

/*******************************************************************************
* main - The main for Arduino API
*
* Calls the user's setup function then runs the user's loop function indefinitely.
*
* First break point for Helix App Cloud Debugger
*
* Returns:  void
*/
void main(void) {
    setup();

	while (1)
	{
		loop();
	}
}


/*******************************************************************************
* tone - Creates a tone
*
* Toggles the output of a digital pin from high to low a specific freqency and
* duration to create a tone.
*
* Arguments: pin       - the pin to toggle
*            freqeuncy  - frequency in Hertz
*            duration  - duration in microseconds
*
* Returns:  void
*/
void tone(uint8_t pin, unsigned int frequency, unsigned long duration)
{
	int i;
	unsigned int period;
	
	/* period in us */
	period = 1000000 / frequency;
	
	for (i = 0; i < duration * 1000L; i += period) {
		digitalWrite(pin, HIGH);
		delayMicroseconds(period >> 1);
		digitalWrite(pin, LOW);
		delayMicroseconds(period >> 1);
	}
}

/*******************************************************************************
* delay - delay in milliseconds
*
*
* Arguments: ms       - delay value
*
* Returns:  void
*/
void delay(unsigned long ms)
{
	sys_thread_busy_wait(ms*1000);
}

/*******************************************************************************
* delay - delay in microseconds
*
*
* Arguments: ms       - delay value
*
* Returns:  void
*/
void delayMicroseconds(unsigned int us)
{
	sys_thread_busy_wait(us);
}


/*******************************************************************************
* map - maps the input value to another output value range
*
* Arguments: value     - input value
*            fromLow   - low end of source range
*            fromHigh  - high end of source range
*            toLow     - low end of destination range
*            toHigh    - high end of destination range
*
* Returns:  uint32_t - mapped value
*/
uint32_t map(uint32_t value, uint32_t fromLow, uint32_t fromHigh, \
             uint32_t toLow,uint32_t toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


