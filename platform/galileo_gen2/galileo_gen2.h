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

typedef struct header {

  signal enum {
    uart0_rx = 0x01,
    uart0_tx = 0x02,
    uart1_rx = 0x04,
    uart2_tx = 0x08
    digital  = 0x0A,
    analog   = 0x0C,
    pwm =    = 0x0E,
    i2c_sda  = 0x10,
    i2c_scl  = 0x20,
    spi_mosi = 0x40,
    spi_miso = 0x80,
    spi_scl  = 0xA0,
    spi_ss   = 0xC0
  };



} arduino_pins;


/* device bindings */

extern struct device *gpio07;
extern struct device *gpioLegacy;
extern struct device *gpio_sus;
extern struct device *gpioEXP0;
extern struct device *gpioEXP1;
extern struct device *gpioEXP2;
extern struct device *i2c;
extern struct device *pinmux;

/* function declarations */

int setup (void);

int gpioOutputSet (int outputPin, int value);
int gpioInputGet (int inputPin);


