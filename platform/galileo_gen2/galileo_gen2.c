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

#include <zephyr.h>
#include <gpio.h>
#include <i2c.h>
#include <pwm.h>
#include <pinmux.h>
#include <pinmux/pinmux.h>
#include <adc.h>
#include <microkernel/ticks.h>
#include <string.h>


#include "arduino.h"
#include "rgb_lcd.h"

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#define PRINT           printf
#else
#include <misc/printk.h>
#define PRINT           printk
#endif

#define P0(pin) (pin)
#define P1(pin) ((pin)+8)

struct device *gpio07;
struct device *gpioLegacy;
struct device *gpio_sus;
struct device *gpioEXP0;
struct device *gpioEXP1;
struct device *gpioEXP2;
struct device *gpioPWM;
struct device *i2c;
struct device *adc;
struct device *pinmux;

#define NUM_IO_PINS_USED 3


/* Buffer holds ADC conversion result */
uint16_t seq_buffer[1] = { 0 };


/*
 * Setup ADC conversion on channel A0.
 * We are only reading one channel so buffer contains one value.
 * minimal delay before start of the conversion (avoiding zero)
 */

struct adc_seq_entry sample[] = {
    {
        .sampling_delay = 1,
        .channel_id     = 0,
        .buffer         = (uint8_t*)seq_buffer,
        .buffer_length  = 1,
    },
};

struct adc_seq_table table =
{
    .entries     = sample,
    .num_entries = 1,
};

void adcCallback
    (
    struct device *dev,
    enum adc_callback_type cb_type
    );

/*
 * @brief Callback, which is invoked when ADC gets new data
 *
 * @param dev ADC device structure
 * @param cb_type can be ADC_CB_DONE or ADC_CB_ERROR
 */
void adcCallback
    (
    struct device *dev,
    enum adc_callback_type cb_type
    )
{
    if (dev == adc && cb_type == ADC_CB_DONE)
    {
        isr_event_send(ADCREADY);
    }
}


/*
 * @brief PWM Pin Mapping configuration
 *
 */
static struct _pwm_map {
	uint8_t pin;
	uint8_t pwm;
} pwm_map[] =
{
	{ 3 , 1  },
	{ 5 , 3  },
	{ 6 , 5  },
	{ 9 , 7  },
	{ 10, 11 },
	{ 11, 9  },
};

/***************************************************************************************************
 *
 * Missing APIs for PWM control
 *
 **************************************************************************************************/

const uint8_t pwm_map_size = sizeof(pwm_map) / sizeof(struct _pwm_map);
// We need a PWM duty cycle API that takes a 8 bit value
#define HARDCODED_CONVERSION(val) (map(val, 0, 255, 0, 4095)) // Oops I peeked in the driver file and saw it was 12 bits

#define pwm_pin_set_duty_cycle_by_8_bit_value(dev,pwm,value)  \
     ((struct pwm_driver_api *)dev->driver_api)->set_values(dev, PWM_ACCESS_BY_PIN, pwm, 0, HARDCODED_CONVERSION(value))


struct pwm_pca9685_config {
        /** The master I2C device's name */
        const char * const i2c_master_dev_name;

        /** The slave address of the chip */
        uint16_t i2c_slave_addr;
};

/** Runtime driver data */
struct pwm_pca9685_drv_data {
        /** Master I2C device */
        struct device *i2c_master;
};
#define MODE1 0x00
#define SLEEP 0x10
#define PRESCALE 0xFE
int pwm_set_frequency(struct device *dev, uint32_t freq)
{
        const struct pwm_pca9685_config * const config = dev->config->config_info;
        struct pwm_pca9685_drv_data * const drv_data = (struct pwm_pca9685_drv_data * const)dev->driver_data;
        struct device * const i2c_master = drv_data->i2c_master;
        uint16_t i2c_addr = config->i2c_slave_addr;
        uint8_t mode1;
        uint8_t buf[] = { MODE1, 0 };

        i2c_write(i2c_master, buf, 1, i2c_addr);
        i2c_read(i2c_master, &buf[1], 1, i2c_addr);
        mode1 = buf[1];

        buf[1] |= SLEEP;

        i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);

        buf[0] = PRESCALE;
        buf[1] =  ((25000000 / (4096 * freq) ));
        i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);

        buf[0] = MODE1;
        buf[1] = mode1;
       return i2c_write(i2c_master, buf, sizeof(buf), i2c_addr);
}
/***************************************************************************************************
 *
 * Grove LCD impelemtation
 *
 **************************************************************************************************/
uint8_t init_lcd_mode = 0;

/*******************************************************************************
* groveLcdCursorSet - Positions the Grove LCD display cursor
*
* Position the Grove LCD display cursor in the row and column requested.
* Note that it is possible to position the cursor outside of visible range.
* 
* Arguments: row    - byte (visible range 0-1) set cursor row position
*            column - byte (visible range 0-15) set cursor column position
* 
* Returns:  void
*/
void groveLcdCursorSet (uint8_t row, uint8_t column)
{
    uint8_t buf[] = {0, 0};

    if (i2c == NULL)
        return;

    column = (row == 0 ? (column | LCD_SETDDRAMADDR) :
                         (column | LCD_SETDDRAMADDR | LCD_SETCGRAMADDR));
    buf[1] = column;
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

    
/*******************************************************************************
* groveLcdInit - Initialize the Grove LCD display mode
*
* Configures the Grove LCD display for 2 line display and a visible blinking
* cursor.
*
* Arguments: 
*
* Returns:  void
*/
void groveLcdInit (void)
{
    uint8_t buf[] = {0, 0};

    PRINT("Initialize LCD ... ");

    if (i2c == NULL) {
        PRINT("failed! \n");
        return;
    }

    /* 2 line display */
    buf[1] = (LCD_FUNCTIONSET | LCD_DISPLAYCONTROL);
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    /* Cursor on, blinking on */
    buf[1] = (LCD_DISPLAYCONTROL | LCD_INPUTSET | LCD_CURSORHOME | LCD_CLEAR);
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    PRINT("done\n");
}

    
/*******************************************************************************
* groveLcdClear - Clears the Grove LCD display
* 
* Create a I2C packet with byte 0 set to 0 (Command byte) and byte 1 set to
* LCD_CLEAR
* 
* Arguments: 
* 
* Returns:  void
*/
void groveLcdClear (void)
{
    uint8_t buf[] = {0, LCD_CLEAR};
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
    task_sleep(45 * sys_clock_ticks_per_sec / 1000);
}

    
/*******************************************************************************
* groveLcdWrite - Send a multibyte string to the Display RAM on LCD controller
*
* Create a I2C packet with byte 0 set to 0x40 (DPRAM) and the rest of the
* callers buffer after that.  Send it all off to the LCD I2C controller.
*
* Arguments: 
*            value - array of DPRAM values (See HD44780 docs)
*            len   - Number of bytes in value[]
*
* Returns:  void
*           Exits on error via lcd_err_exit()
*/
void groveLcdWrite ( unsigned char *string, int len)
{
    int i;
    uint8_t buf[] = {LCD_SETCGRAMADDR, 0};

    if (len > 127)
        return;

    for (i = 0; i < len; i++)
        {
        buf[1] = string[i];
        i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
        }

    task_sleep(45 * sys_clock_ticks_per_sec / 1000);

}

/*******************************************************************************
* groveLcdPrint - Display a string on the Grove LCD display
* 
* Displays a given string on the Grove LCD display starting at row and column 
* 
* Arguments: 
*            row    - byte, 0 or 1 for row 0 or row 1
*            column - byte, 0 - 15 for visible columns
*            string - string pointer
*            len    - length of the string to display
* 
* Returns:  void
*/
void groveLcdPrint (char * string)
{
    int i,len=0;
    uint8_t buf[] = {LCD_SETCGRAMADDR, 0};

   // groveLcdCursorSet(row, column);
   
    len = strlen(string);
    for (i = 0; i < len; i++)
        {
        buf[1] = string[i];
        i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
        }
}


/*******************************************************************************
* groveLcdBegin - Initialize the Grove LCD display mode
*
* Configures the Grove LCD display for 2 line display and a visible blinking
* cursor.
*
* Arguments: 
*
* Returns:  void
*/
void groveLcdBegin (uint8_t cols, uint8_t rows)
{
    uint8_t buf[] = {0, 0};

    PRINT("Initialize LCD ... ");

    if (i2c == NULL)
        {
        PRINT("failed! \n");
        return;
        }

    /* 2 line display */

    groveLcdClear();

    buf[0] = cols;
    buf[1] = rows;
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    init_lcd_mode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    buf[0] = 0;
    buf[1] = LCD_INPUTSET | init_lcd_mode;
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    PRINT("done\n");
}


void groveLcdHome()
{
    uint8_t buf[] = {0, LCD_CURSORHOME};
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
    task_sleep(45 * sys_clock_ticks_per_sec / 1000);
}

// Turn the display on/off (quickly)
void groveLcdnoDisplay()
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode &= ~LCD_DISPLAYON;
    buf[1] = LCD_DISPLAYCONTROL | init_lcd_mode;
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

void groveLcddisplay() 
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode |= LCD_DISPLAYON;
    buf[1] = LCD_DISPLAYCONTROL | init_lcd_mode;
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// Turns the underline cursor on/off
void groveLcdnoCursor()
{
    uint8_t buf[] = {0, 0};
    
    //buf[1] = 0xc;       /* Cursor off, blinking off */
    buf[1] = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;	/* cursor on, blinking on */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}


void groveLcdcursor(void) 
{
    uint8_t buf[] = {0, 0};

    //buf[1] = 0xe;       /* Cursor on, blinking on */
    buf[1] = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF;	/* cursor on, blinking on */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// Turn on and off the blinking cursor
void groveLcdnoBlink()
{
    uint8_t buf[] = {0, 0};

    //buf[1] = 0xe;       /* Cursor on, blinking off */
    buf[1] = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKOFF;	/* cursor on, blinking on */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

void groveLcdblink()
{
    uint8_t buf[] = {0, 0};

    buf[1] = LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSORON | LCD_BLINKON;	/* cursor on, blinking on */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// These commands scroll the display without changing the RAM
void groveLcdscrollDisplayLeft(void)
{
    uint8_t buf[] = {0, 0};

    buf[1] = LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT;

    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

void groveLcdscrollDisplayRight(void)
{
    uint8_t buf[] = {0, 0};

    buf[1] = LCD_SHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT;

    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// This is for text that flows Left to Right
void groveLcdleftToRight(void)
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode |= LCD_ENTRYLEFT;
    
    buf[1] = LCD_INPUTSET | init_lcd_mode;	/* left to right */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// This is for text that flows Right to Left
void groveLcdrightToLeft(void)
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode &= ~LCD_ENTRYLEFT;
    
    buf[1] = LCD_INPUTSET | init_lcd_mode;	/* left to right */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// This will 'right justify' text from the cursor
void groveLcdautoscroll(void)
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode |= LCD_ENTRYSHIFTINCREMENT ;
    
    buf[1] = LCD_INPUTSET | init_lcd_mode;	/* auto scroll */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

// This will 'left justify' text from the cursor
void groveLcdnoAutoscroll(void)
{
    uint8_t buf[] = {0, 0};

    init_lcd_mode &= ~LCD_ENTRYSHIFTINCREMENT;
    
    buf[1] = LCD_INPUTSET | init_lcd_mode;	/* left to right */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

void groveLcdcreateChar(uint8_t location, uint8_t charmap[])
{
    uint8_t buf[] = {0, 0};
    unsigned char dta[9];
    
    location &= 0x7; // we only have 8 locations 0-7
    
    buf[1] = LCD_SETCGRAMADDR | (location << 3);
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    dta[0] = 0x40;
    for(int i=0; i<8; i++)
    {
        dta[i+1] = charmap[i];
    }
    i2c_polling_write(i2c, dta, sizeof(dta), LCD_ADDR);
}

/*********** mid level commands, for sending data/cmds */

// send command
/*******************************************************************************
* groveLcdCommand - Send a 1 byte command to the LCD controller
*
* Create a I2C packet with byte 0 set to 0 (Command byte) and byte 1 set to
* the value passed by the user
*
* Arguments: 
*            value - 1 byte command (See HD44780 docs)
*
* Returns:  void
*           Exits on error via lcd_err_exit()
*/
void groveLcdCommand (unsigned char value)
{
    uint8_t buf[] = {0, value};

    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    /*
     * Wait long enough for command to work.  See GroveLCD documents
     */

    switch(value)
        {
        case LCD_CLEAR:        // Wait 1.53 ms (rounded to 1.6 ms)
        case LCD_CURSORHOME:
            task_sleep(1600 * sys_clock_ticks_per_sec / 1000);
            break;

        default:              // Wait 39 usec (rounded up to 40)
            task_sleep(40 * sys_clock_ticks_per_sec / 1000);
            break;
        }
}

// send data
size_t groveLcdwrite(uint8_t value)
{
    uint8_t buf[] = {LCD_SETCGRAMADDR, value};

    return i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);
}

void grove_setReg(unsigned char addr, unsigned char dta)
{
    uint8_t buf[] = {addr, dta};

    i2c_polling_write(i2c, buf, sizeof(buf), RGB_ADDR);
	
}

/*******************************************************************************
* groveLcdColorRGBSet - Sets the backlight color of the Grove LCD display
*
* Create a I2C packet with 2 bytes that are used to access the RGB register
* and assign the red, green and blue values that controls the backlight color
* of the Grove LCD display.
*
* Arguments: i2c   - I2C device structure pointer
*            red   - byte (0-255) controlling color red intensity
*            green - byte (0-255) controlling color green intensity
*            blue  - byte (0-255) controlling color blue intensity
*
* Returns:  void
*/
void groveLcdColorRGBSet (uint8_t red, uint8_t green, uint8_t blue)
{
    uint16_t i2c_addr = RGB_ADDR;
    uint8_t buf[] = {0, 0};

    buf[0] = 0; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 1; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 8; buf[1] = 0xaa;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);

    buf[0] = 4; buf[1] = red;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 3; buf[1] = green;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 2; buf[1] = blue;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
}


const uint8_t grove_color_values[4][3] = 
{
    {255, 255, 255},            // white
    {255, 0, 0},                // red
    {0, 255, 0},                // green
    {0, 0, 255},                // blue
};

void groveLcdSetRGB(unsigned char color)
{
    if(color > BLUE)
	return ;
    
    groveLcdColorRGBSet(grove_color_values[color][0],grove_color_values[color][1],grove_color_values[color][2]);
}

void groveLcdsetPWM (unsigned char color, unsigned char pwm)
{
    grove_setReg(color, pwm);
}

void groveLcdsetColorAll (void)
{
    uint16_t i2c_addr = RGB_ADDR;
    uint8_t buf[] = {0, 0};

    buf[0] = 0; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 1; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 8; buf[1] = 0xaa;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);

    buf[0] = 4; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 3; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
    buf[0] = 2; buf[1] = 0;
    i2c_polling_write(i2c, buf, sizeof(buf), i2c_addr);
}

void groveLcdsetColorWhite(void)
{
    groveLcdSetRGB(WHITE);
}

/*
 * Function Pointer assignment 
 */
rgb_lcd lcd = { 
			groveLcdBegin, 
			groveLcdClear, 
			groveLcdHome, 
			groveLcdnoDisplay, 
			groveLcddisplay, 
			groveLcdnoBlink, 
			groveLcdblink, 
			groveLcdnoCursor, 
			groveLcdcursor, 
			groveLcdscrollDisplayLeft, 
			groveLcdscrollDisplayRight, 
			groveLcdleftToRight, 
			groveLcdrightToLeft, 
			groveLcdautoscroll, 
			groveLcdnoAutoscroll, 
			groveLcdcreateChar, 
			groveLcdCursorSet , 
			groveLcdwrite, 
			groveLcdCommand, 
			groveLcdColorRGBSet, 
			groveLcdsetPWM, 
			groveLcdSetRGB, 
			groveLcdsetColorAll, 
			groveLcdsetColorWhite,
			groveLcdPrint
};



/***************************************************************************************************
 *
 * Interface implementation
 *
 **************************************************************************************************/
/*
 * gpioOutputSet - set the output value of a pin
 *
 * outputPin: Arduino connector pin number
 * value: 0=set output to low, otherwise set high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */

int gpioOutputSet (int outputPin, int value)
{
    int rc1 = DEV_OK;
    switch (outputPin)
        {
        case 0:
            /* gpio11=signal */
            gpio_pin_write(gpio07, 3, value);
            break;
        case 1:
            /* gpio12=signal */
            gpio_pin_write(gpio07, 4, value);
            break;
        case 2:
            /* gpio13=signal */
            gpio_pin_write(gpio07, 5, value);
            break;
        case 3:
            /* gpio14=signal */
            gpio_pin_write(gpio07, 6, value);
            break;
        case 4:
            /* gpio6=signal */
            gpio_pin_write(gpio_sus, 4, value);
            break;
        case 5:
            /* gpio0=signal */
            gpio_pin_write(gpioLegacy, 0, value);
            break;
        case 6:
            /* gpio1=signal */
            gpio_pin_write(gpioLegacy, 1, value);
            break;
        case 7:
            /* gpio38=signal */
            gpio_pin_write(gpioEXP1, P0(6), value);
            break;
        case 8:
            /* gpio40=signal*/
            gpio_pin_write(gpioEXP1, P1(0), value);
            break;
        case 9:
            /* gpio4=signal */
            gpio_pin_write(gpio_sus, 2, value);
            break;
        case 10:
            /* gpio10=signal */
            gpio_pin_write(gpio07, 2, value);
            break;
        case 11:
            /* gpio5=signal */
            gpio_pin_write(gpio_sus, 3, value);
            break;
        case 12:
            /* gpio15=signal*/
            gpio_pin_write(gpio07, 7, value);
            break;
        case 13:
            /* gpio7=signal */
            gpio_pin_write(gpio_sus, 5, value);
            break;

        default:
            rc1 = DEV_FAIL;
            break;
        }

    if (rc1 != DEV_OK)
        {
        PRINT("GPIO set error %d!!\n", rc1);
        }

    return rc1;
}

/*
 * gpioInputGet - get value of a pin
 *
 * inputPin: Arduino connector pin number
 * Returns: 0 if value is low,
 *          1 if value is high
 *
 * Note: comments below "gpioXX" refer to the Linux GPIO naming convention
 */

int gpioInputGet (int inputPin)
{
    int value;
    int rc1 = DEV_OK;
    switch (inputPin)
        {
        case 0:
            /* gpio11=signal */
            gpio_pin_read(gpio07, 3, &value);
            break;
        case 1:
            /* gpio12=signal */
            gpio_pin_read(gpio07, 4, &value);
            break;
        case 2:
            /* gpio13=signal*/
            gpio_pin_read(gpio07, 5, &value);
            break;
        case 3:
            /* gpio14=signal */
            gpio_pin_read(gpio07, 6, &value);
            break;
        case 4:
            /* gpio6=signal */
            gpio_pin_read(gpio_sus, 4, &value);
            break;
        case 5:
            /* gpio0=signal*/
            gpio_pin_read(gpioLegacy, 0, &value);
            break;
        case 6:
            /* gpio1=signal */
            gpio_pin_read(gpioLegacy, 1, &value);
            break;
        case 7:
            /* gpio38=signal */
            gpio_pin_read(gpioEXP1, P0(6), &value);
            break;
        case 8:
            /* gpio40=signal */
            gpio_pin_read(gpioEXP1, P1(0), &value);
            break;
        case 9:
            /* gpio4=signal */
            gpio_pin_read(gpio_sus, 2, &value);
            break;
        case 10:
            /* gpio10=signal */
            gpio_pin_read(gpio07, 2, &value);
            break;
        case 11:
            /* gpio5=signal */
            gpio_pin_read(gpio_sus, 3, &value);
            break;
        case 12:
            /* gpio15=signal */
            gpio_pin_read(gpio07, 7, &value);
            break;
        case 13:
            /* gpio7=signal */
            gpio_pin_read(gpio_sus, 5, &value);
            break;
        default:
            rc1 = DEV_FAIL;
            break;
        }

    if (rc1 != DEV_OK)
        {
        PRINT("GPIO read error %d!!\n", rc1);
        }

    if (0 == value)
        return 0;
    else
        return 1;

}


void pinMode(uint8_t pin, uint8_t mode) {

    DBG_PRINT("Initialized pin%d\n", pin);

    /* set direction  */
    switch (mode) {
	case INPUT_PULLUP:
    	  DBG_PRINT("Setting mode to INPUT_PULLUP\n");
	case INPUT:
    	DBG_PRINT("Setting mode INPUT\n");
           if (pinmux_set_pin(pinmux, pin, PINMUX_FUNC_B)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
	break;
        case OUTPUT:
    	   DBG_PRINT("Setting mode to OUTPUT...");
           if (pinmux_set_pin(pinmux, pin, PINMUX_FUNC_A)) {
    	      DBG_PRINT("failed\n");
           } else {
    	      DBG_PRINT("success\n");
           }
    }
}

void digitalWrite(register uint8_t pin, register uint8_t value)
{


    DBG_PRINT("Initialized pin%d\n", pin);

    if ( value == HIGH )
    	DBG_PRINT("Setting pin to HIGH...");
    else
    	DBG_PRINT("Setting direction to LOW...");
    if (gpioOutputSet(pin, value)) { 
    	DBG_PRINT("Failed\n");
    } else {
    	DBG_PRINT("Success\n");
    }
}

int digitalRead(uint_t pin)
{
	uint8_t val;

	val = gpioInputGet (pin);
        DBG_PRINT("Gpio%d is %d\n", pin, val);

	return val;
}
	
void analogWrite(uint32_t pin, uint32_t value)
{
	
    uint8_t pwm = 0xFF;
    uint8_t i = 0;

    DBG_PRINT("Initialized pin%d\n", pin);
    if ( ((pin) == 3 || (pin) == 5 || (pin) == 6 || (pin) == 9 || (pin) == 10 ||
		(pin) == 11 )) {
	    DBG_PRINT("GPIO pin %d\n", pin);
	    /* find the PWM mapping */
	    for (i = 0; i < pwm_map_size; i++) {
		    if (pin == pwm_map[i].pin)
			    pwm = pwm_map[i].pwm;
	    }
	    if (pwm == 0xFF) {
		    DBG_PRINT("%s bad pwm pin %d\n", __FUNCTION__, pin);
		    return ;
	    }
		
	    DBG_PRINT("Writing [%d] to PWM pin [%d]\n",value,pwm);
	    if (pinmux_set_pin(pinmux, pin, PINMUX_FUNC_C)) {
		    DBG_PRINT("failed\n");
	    } else {
		    DBG_PRINT("success\n");
	    }
	    pwm_pin_set_duty_cycle_by_8_bit_value(gpioPWM,pwm,value);
    }
    else if ( ((pin) == 14 || (pin) == 15 || (pin) == 16 || (pin) == 17 || (pin) == 18 ||
			    (pin) == 19 )) {
	    DBG_PRINT("ADC pin %d\n", pin);
	    DBG_PRINT("   -- NOT SUPPORTED  -- ADC pin %d\n", pin);
    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
    }
}

uint16_t analogRead(uint8_t pin)
{
    uint16_t val=0;
    int rc;
 
    DBG_PRINT("Initialized pin%d\n", pin);
    if ( ((pin) == 3 || (pin) == 5 || (pin) == 6 || (pin) == 9 || (pin) == 10 ||
		(pin) == 11 )) {
	    DBG_PRINT("PWM pin %d\n", pin);
	    DBG_PRINT("   -- NOT SUPPORTED  -- PWM pin %d\n", pin);
    }
    else if ( ((pin) == 14 || (pin) == 15 || (pin) == 16 || (pin) == 17 || (pin) == 18 ||
			    (pin) == 19 )) {
	    DBG_PRINT("ADC pin %d\n", pin);
	    DBG_PRINT("reading ADC...\n");
	    sample[0].channel_id = 0;
	    rc = adc_read(adc, &table);
	    if (DEV_OK != rc)
	    {
		    PRINT("ADC read error! (%d)\n", rc);
	    }
	    else
	    {
		    DBG_PRINT("wait for callback\n");
		    task_event_recv_wait(ADCREADY);
	    }
	    val = seq_buffer[0];
    }
    else {
	    DBG_PRINT("INVALID Operation for pin %d\n", pin);
    }
    return val;
}

void analogReference(uint8_t type)
{
}


void hardware_init() 
{
    int status;
    uint8_t buf[] = {0, 0};

    /* get the device bindings */

    /* On-chip Quark GPIO */
    gpio07 = device_get_binding(CONFIG_GPIO_DW_0_NAME);            // GPIO[7:0]
    gpioLegacy = device_get_binding(CONFIG_GPIO_MMIO_0_DEV_NAME);  // GPIO[9:8]
    gpio_sus = device_get_binding(CONFIG_GPIO_MMIO_1_DEV_NAME);  //GPIO_SUS[5:0]

    /* external components */
    gpioEXP0 = device_get_binding(CONFIG_GPIO_PCAL9535A_0_DEV_NAME);
    gpioEXP1 = device_get_binding(CONFIG_GPIO_PCAL9535A_1_DEV_NAME);
    gpioEXP2 = device_get_binding(CONFIG_GPIO_PCAL9535A_2_DEV_NAME);

    /* i2c master of gpioEXP0/1/2 */
    i2c = device_get_binding(CONFIG_GPIO_PCAL9535A_1_I2C_MASTER_DEV_NAME);

    /* PWM */
    gpioPWM  = device_get_binding(CONFIG_PWM_PCA9685_0_DEV_NAME);
    adc  = device_get_binding(CONFIG_ADC_TI_ADC108S102_0_DRV_NAME);
    pwm_set_frequency(gpioPWM, PWM_FREQUENCY);

    pinmux = device_get_binding(PINMUX_NAME);

    if (!gpio07)
        {
        PRINT("GPIO DW not found!!\n");
        }

    if (!gpioLegacy)
        {
        PRINT("GPIO MMIO 0 not found!!\n");
        }

    if (!gpio_sus)
        {
        PRINT("GPIO MMIO 1 not found!!\n");
        }

    if (!gpioEXP0)
        {
        PRINT("EXP0 not found!!\n");
        }

    if (!gpioEXP1)
        {
        PRINT("EXP1 not found!!\n");
        }

    if (!gpioEXP2)
        {
        PRINT("EXP2 not found!!\n");
        }

    if (!i2c)
        {
        PRINT("I2C not found!!\n");
        }

    if (!gpioPWM)
        {
        PRINT("PWM not found!!\n");
	}

    if (!adc)
        {
        PRINT("ADC not found!!\n");
	}

    if (!pinmux)
        {
        PRINT("Pinmux not found!!\n");
        }

    if (!(gpio07 && gpioLegacy && gpio_sus && gpioEXP0 &&
          gpioEXP1 && gpioEXP2 && i2c && pinmux))
        {
        PRINT("Stopped.\n");
        return;
        }

    status = i2c_configure(i2c, (I2C_SPEED_FAST << 1) | I2C_MODE_MASTER);

    if (status != DEV_OK)
        {
        PRINT("I2C configuration error: %d Stopped.\n", status);
        return;
        }

    adc_set_callback(adc, adcCallback);

    /* Initialize LCD for future Use */
    buf[1] = 0x38;      /* 2 line display */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR);

    buf[1] = 0xe;       /* Cursor on, blinking on */
    i2c_polling_write(i2c, buf, sizeof(buf), LCD_ADDR); 

    /* Initialize LCD */
    //groveLcdInit();

    return;
}
