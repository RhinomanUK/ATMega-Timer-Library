/*
ArduTimers.h


Library routines for expanded timer functions for an Arduino Mega,
uses the Waveform generator module, resolution is up to 16-bit.
includes input capture and output compare functions
8-bit timers 0 and 2 not supported in this release

refer to ATMega2560 datasheet for details on timer operations

James Holland
Version 1 - 28th December 2020

*/

#ifndef ARDU_TIMER_H
#define ARDU_TIMER_H

#include <Arduino.h>

#define TMR0	0
#define TMR1	1
#define TMR2	2
#define TMR3	3
#define TMR4	4
#define TMR5	5


// defines for 8/9/10-bit PWM modes
#define  PWM_8BIT   1
#define  PWM_9BIT   2
#define  PWM_10BIT  3

// define compare values for high res PWM
#define  PWM_11BIT  2047
#define  PWM_12BIT  4095
#define  PWM_13BIT	8191
#define  PWM_14BIT  16383
#define	 PWM_15BIT  32767

// Output Compare/PWM Pin definitions
#define D2  2   // OCR3B
#define D3  3   // OCR3C
#define D4  4   // OCR0B    // timer0 is used for Arduino system timer!
#define D5  5   // OCR3A    // also AIN1 !!!!!
#define D6  6   // OCR4A
#define D7  7   // OCR4B
#define D8  8   // OCR4C
#define D9  9   // OCR2B    // 8-bit only
#define D10 10  // OCR2A    // 8-bit only
#define D11 11  // OCR1A
#define D44 44  // OCR5C
#define D45 45  // OCR5B
#define D46 46  // OCR5A


// Input Capture pin definitions
// no IC on timer0 or timer2
#define D49  49   // ICR4
#define D48  48   // ICR5


// Arduino doesn't bring the following channels out to pins 
// but give then dummy pin numbers for future proofing
#define D100    100		// OC1B
#define D101    101		// OC1C
#define D102	102		// ICR1
#define D103	103	    // ICR3


// PWM Mode definitions
#define PWM_OFF       0       // normal timer operation - mode 0
#define PWM_PC        1       // phase correct PWM, modes 1,2,3
#define PWM_PC_IC     2		  // phase correct PWM, reset on input capture - mode 
#define PWM_PC_OC     3
#define PWM_CTC_IC    4       // Clear Timer on Compare, mode 12 
#define PWM_CTC_OC    5       // Clear Timer on Compare, mode 4
#define PWM_FAST      6       // fast PWM, modes 5,7
#define PWM_FAST_IC   7
#define PWM_FAST_OC   8
#define PWM_PFC_IC    9       // PWM phase and frequency correct
#define PWM_PFC_OC    10


// clock divider definitions - assumes a standard Arduino Mega at 16MHz
#define CLK_OFF     0x00       // Counter stopped
#define CLK_16M     0x01       // clock/1 = 16MHz
#define CLK_2M      0x02       // clock/8 = 2MHz
#define CLK_250k    0x03
#define CLK_62k5    0x04
#define CLK_15625k  0x05
#define CLK_EXT_FE  0x06       // external clock falling edge
#define CLK_EXT_FR  0x07


// bit definitions for pwm mode control
#define WGMn3     4
#define WGMn2     3
#define WGMn1     1
#define WGMn0     0

// define output pin modes
#define   NORMAL        0x00
#define   TOGGLE        0x04
#define   CLR_ON_CMP    0x08
#define   SET_ON_CMP    0x0C
#define   CLR_ON_CMP_UP 0x08
#define   SET_ON_CMP_DN 0x08
#define   SET_ON_CMP_UP 0x0C
#define   CLR_ON_CMP_DN 0x0C

#define OCRnA_OFF   0x3F		// set pin mode to normal
#define OCRnB_OFF   0xCF		// set pin mode to normal
#define OCRnC_OFF   0xF3		// set pin mode to normal


// define input capture modes
#define FILT_ON		0x80
#define FILT_OFF	0x00
#define RIS_EDG		0x40
#define FALL_EDG	0x00



class ArduTimer{
	
	private:
    // typedef void (*intFuncPtr)(int); //typedef assigns a name (inFuncPtr) to the pointer type
    static int doNothing(void);
	
	public:
		uint8_t initTimer(uint8_t timer, uint16_t pwm_mode, uint8_t clk_mode, uint16_t resolution);
		uint8_t connectOCPin(uint8_t oc_pin, uint8_t pin_mode);
		uint8_t disconnectOCPin(uint8_t oc_pin);
		uint8_t setOCDutyCycle(uint8_t oc_pin, uint16_t pulseWidth);
		uint8_t enableOCInterrupt(uint8_t oc_pin);
		uint8_t disableOCInterrupt(uint8_t oc_pin);
		uint8_t connectICPin(uint8_t filter_mode, uint8_t trig_edge);	//there is only one IC channel per timer so no need to define the pin
		uint16_t getICValue(uint16_t *ICCount);

	private:
		
		uint8_t local_TMR_ch;			// store timer channel to allow checks for valid pin numbers
};

	
#endif
