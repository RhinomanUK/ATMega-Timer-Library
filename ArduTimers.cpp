/*
ArduTimers.cpp
  
 
  Library routines for expanded timer functions for an Arduino Mega,
  uses the Waveform generator module, resolution is up to 16-bit.
  includes input capture and output compare functions

  Refer to ArduTimersr.h for #defines for setting up and calling functions
  refer to ATMega2560 datasheet for details on timer operations

    James Holland
    Version 1 - 28th December 2020

*/

#include <Arduino.h>
#include "ArduTimers.h"


//#include <avr/interrupt.h>      //ISR() must be defined elsewhere if interrupts are used !!!!!)


uint8_t ArduTimer::initTimer(uint8_t timer, uint16_t pwm_mode, uint8_t clk_mode, uint16_t resolution)
{

  uint8_t error = 0;
  volatile uint8_t* TCCRAAddress;
  volatile uint8_t* TCCRBAddress;
  volatile uint16_t* OCRnAddress;
  volatile uint16_t* ICRnAddress;
  
  uint16_t newOCRValue = 0;
  uint16_t newICRValue = 0;
  uint8_t newTCCRAValue = 0;
  uint8_t newTCCRBValue = 0; 

  local_TMR_ch = timer;

  switch (timer) {
      case TMR1: TCCRAAddress = &TCCR1A;
				TCCRBAddress = &TCCR1B;
				OCRnAddress  = &OCR1A;
				ICRnAddress  = &ICR1;
				break;
      case TMR2: TCCRAAddress = &TCCR2A;
				TCCRBAddress = &TCCR2B;
				// OCRnAddress  = &OCR2A;
				// ICRnAddress  = &ICR2;		//no IC on timer2
				break;
      case TMR3: TCCRAAddress = &TCCR3A;
               TCCRBAddress = &TCCR3B;
               OCRnAddress  = &OCR3A;
               ICRnAddress  = &ICR3; 
               break;   
	  case TMR4: TCCRAAddress = &TCCR4A;
				TCCRBAddress = &TCCR4B;
				OCRnAddress  = &OCR4A;
				ICRnAddress  = &ICR4;
	  case TMR5:  TCCRAAddress = &TCCR5A;
				TCCRBAddress = &TCCR5B;
				OCRnAddress  = &OCR5A;
				ICRnAddress  = &ICR5;
				break; 
   default:  error =1;
              break;
  } // end switch



  if(pwm_mode==PWM_PC){
     newTCCRAValue |= resolution; 
  }
  else if(pwm_mode==PWM_PC_IC){
    newTCCRAValue |= (1<<WGMn1);
    newTCCRBValue |= (1<<WGMn3);
  }
  else if(pwm_mode==PWM_PC_OC){
    newTCCRAValue |= (1<<WGMn1)|(1<<WGMn0);
    newTCCRBValue |= (1<<WGMn3);
  }
  else if(pwm_mode==PWM_FAST){
    newTCCRAValue |= resolution;
    newTCCRBValue |= (1<<WGMn2);
  }
  else if(pwm_mode==PWM_FAST_IC){
    newTCCRAValue |= (1<<WGMn1);
    newTCCRBValue |= (1<<WGMn3)|(1<<WGMn2);
    if(resolution==PWM_14BIT){
        newICRValue = PWM_14BIT;
    }
    else{
      newICRValue = PWM_12BIT;
    }
  }
  else if(pwm_mode==PWM_FAST_OC){
    newTCCRAValue |= (1<<WGMn1)|(1<<WGMn0);
    newTCCRBValue |= (1<<WGMn3)|(1<<WGMn2);
    newOCRValue = resolution; 
  }
  else if(pwm_mode==PWM_CTC_IC){
    newTCCRBValue |= (1<<WGMn3)|(1<<WGMn2); 
    newICRValue = resolution;
  } 
  else if(pwm_mode==PWM_CTC_OC){
    newTCCRBValue |= (1<<WGMn2);
    newOCRValue = resolution;
  }

  newTCCRBValue |= clk_mode;      // clock source/divide

  *TCCRAAddress = newTCCRAValue;    // update the control registers
  *TCCRBAddress = newTCCRBValue;
  *OCRnAddress = newOCRValue;
  *ICRnAddress = newICRValue;

  return error;
}


uint8_t ArduTimer::connectOCPin(uint8_t PWMPin, uint8_t pin_mode){

	// enable output on OC pin and set DDR

	uint8_t error = 0;

	switch (PWMPin) {
		case D2: TCCR3A &= OCRnB_OFF; 
				TCCR3A |= (pin_mode * 4);		// OCR3B
				break;
		case D3: TCCR1A &= OCRnC_OFF; 
				TCCR1A |= (pin_mode);			// OCR3C
				break;
		case D4: TCCR0A &= OCRnB_OFF;			// OCR0B ???????
				TCCR0A |=(pin_mode * 4);		
				break;
		case D5: TCCR3A &= OCRnA_OFF; 
				TCCR3A |= (pin_mode * 16);    // OCR3A
				break;
		case D6: TCCR4A &= OCRnA_OFF; 
				TCCR4A |= (pin_mode * 16);    // OCR4A
				break;
		case D7: TCCR4A &= OCRnB_OFF; 
				TCCR4A |= (pin_mode * 4);		// OCR4B
				break;
		case D8: TCCR4A &= OCRnC_OFF;
				TCCR4A |= (pin_mode);			// OCR4C
				break;
		case D9:	TCCR2A &= OCRnB_OFF;	
				TCCR2A |= (pin_mode * 4);		 // OCR2B ????
				break;
		case D10: TCCR2A &= OCRnA_OFF;
				TCCR2A |= (pin_mode * 16);		// OCR2A ????
				break;
		case D11: TCCR1A &= OCRnA_OFF;
				TCCR1A |=(pin_mode * 16);		// OCR1A
				break;
		case D44: TCCR5A &= OCRnC_OFF;
				TCCR5A |= (pin_mode);			// OCR5C
				break;
		case D45:TCCR5A &= OCRnB_OFF; 
				TCCR5A |= (pin_mode * 4);		// OCR5B
				break;
		case D46: TCCR5A &= OCRnA_OFF;
				TCCR5A |= (pin_mode * 16);    // OCR5A
				break;
		default: error = 1;
	} // end switch

	return error;
}

uint8_t ArduTimer::disconnectOCPin(uint8_t oc_pin){

uint8_t error = 0;

//********************** pin numbers - channels !!! **************************

  switch (oc_pin) {
    case D2:  TCCR3A &= OCRnB_OFF;    // OCR3B
              break;
    case D3:  TCCR1A &= OCRnC_OFF;    // OCR3C
              break;
    case D4:  TCCR0A &= OCRnB_OFF;    // OCR0B ???????
              break;            
    case D5:  TCCR3A &= OCRnA_OFF;    // OCR3A
              break;    
    case D6:  TCCR4A &= OCRnA_OFF;    // OCR4A
              break;
    case D7:  TCCR4A &= OCRnB_OFF;    // OCR4B
              break;
    case D8:  TCCR4A &= OCRnC_OFF;    // OCR4C
              break;     
    case D9:  TCCR2A &= OCRnB_OFF;    // OCR2B
              break;
    case D10: TCCR2A &= OCRnA_OFF;    // OCR2A
              break;              
    case D11: TCCR1A &= OCRnA_OFF;    // OCR1A
              break;
    case D44: TCCR5A &= OCRnC_OFF;    // OCR5C
              break;              
    case D45: TCCR5A &= OCRnB_OFF;    // OCR5B non functional????
              break;
    case D46: TCCR5A &= OCRnA_OFF;    // OCR5A
              break;              
    default: error = 1;
  } // end switch

  return error; 



  
}

uint8_t ArduTimer::setOCDutyCycle(uint8_t pwm_pin, uint16_t pulseWidth)
{
  uint8_t error = 0;

  switch (pwm_pin) {
    case 2:  OCR3B = pulseWidth;
              break;
    case 3:  OCR3C = pulseWidth;
              break;
    case 4:  OCR0B = pulseWidth;
              break;            
    case 5:  OCR3A = pulseWidth;
              break;    
    case 6:  OCR4A = pulseWidth;
              break;
    case 7:  OCR4B = pulseWidth;
              break;
    case 8:  OCR4C = pulseWidth;
              break;     
    case 9:  OCR2B = pulseWidth;
              break;
    case 10: OCR2A = pulseWidth;
              break;              
    case 11: OCR1A = pulseWidth;
              break;
    case 44: OCR5C = pulseWidth;
              break;              
    case 45: OCR5B = pulseWidth;      // non functional????
              break;
    case 46: OCR5A = pulseWidth;
              break;              
    default: error = 1;
  } // end switch


  return error;
}



uint8_t ArduTimer::connectICPin(uint8_t filter_mode, uint8_t trig_edge)
{
	
	uint8_t error = 0;
	uint8_t tempTCCRB = 0;

	tempTCCRB += filter_mode;
	tempTCCRB += trig_edge;

	if(local_TMR_ch==1){
		TCCR4B &= 0x3F;
		TCCR4B |= tempTCCRB;
		}
	else if(local_TMR_ch==3){
		TCCR5B &= 0x3F;
		TCCR5B |= tempTCCRB;
		}
	else if(local_TMR_ch==4){
		TCCR4B &= 0x3F;
		TCCR4B |= tempTCCRB;
		}
	else if(local_TMR_ch==5){
		TCCR5B &= 0x3F;
		TCCR5B |= tempTCCRB;
		}
	else{
		error = 1;
		}

}


uint16_t ArduTimer::getICValue(uint16_t *ICCount)
{

	
	uint8_t error = 0;
	uint8_t tempICR = 0;


	if(local_TMR_ch==1){
		tempICR = ICR1;
	}
	else if(local_TMR_ch==3){
		tempICR = ICR3
		;
	}
	else if(local_TMR_ch==4){
		tempICR = ICR4;
	}
	else if(local_TMR_ch==5){
		tempICR = ICR5;
	}
	else{
		error = 1;
	}

	*ICCount = tempICR;
}




int ArduTimer::doNothing(void) {

  // this is a dummy routine that traps any unintended interrupt calls.
  // this needs to be moved to a separate routine with all other interrupt routines
  // All outputs should be driven to a safe state and then the code loops until 
  // the watchdog resets the micro.

  // while(1); wait for watchdog
  return 1;
  
}


//enable interrupts
uint8_t ArduTimer::enableOCInterrupt(uint8_t oc_pin) {

  uint8_t error = 0;
/*  
  int (*intFuncPtr)();      // typedef assigns a name (inFuncPtr) to the pointer type
  intFuncPtr = doNothing;   // set up a pointer to the doNothing function
*/
  switch (oc_pin) {
    case 2:
      TIMSK3 = (1 << OCIE3B);
      break;
    case 3:
      TIMSK3 = (1 << OCIE3C);
      break;
    case 4:
      TIMSK0 = (1 << OCIE0B);
      break;      
    case 5:
      TIMSK3 = (1 << OCIE3A);
      break;      
    case 6:
      TIMSK4 = (1 << OCIE4A);
      break;
    case 7:
      TIMSK4 = (1 << OCIE4B);
      break; 
    case 8:
      TIMSK4 = (1 << OCIE4C);
      break;  
    case 9:
      TIMSK2 = (1 << OCIE2B);
      break; 
    case 10:
      TIMSK2 = (1 << OCIE2A);
      break;
    case 11:
      TIMSK1 = (1 << OCIE1A);
      break;
    case 44:
      TIMSK5 = (1 << OCIE5C);
      break; 
    case 45:
      TIMSK5 = (1 << OCIE5B);
      break;
    case 46:
      TIMSK5 = (1 << OCIE5A);
      break;
   default:
      error = 1;
      break;
  } // end switch

      return error;
}




//disable interrupts
uint8_t ArduTimer::disableOCInterrupt(uint8_t oc_pin) {
  //clear the interrupt enable bit

  uint8_t error = 0;
  
  switch (oc_pin) {
    case 2:
      TIMSK3 = (0 << OCIE3B);
      break;
    case 3:
      TIMSK3 = (0 << OCIE3C);
      break;
    case 4:
      TIMSK0 = (0 << OCIE0B);
      break;
    case 5:
      TIMSK3 = (0 << OCIE3A);   // also AIN1 !!
      break;
    case 6:
      TIMSK4 = (0 << OCIE4A);
      break;
    case 7:
      TIMSK4 = (0 << OCIE4B);
      break;
    case 8:
      TIMSK4 = (0 << OCIE4C);
      break;
    case 9:
      TIMSK2 = (0 << OCIE2B);  
      break;
    case 10:
      TIMSK2 = (0 << OCIE2A);   
      break;
    case 11:
      TIMSK1 = (0 << OCIE1A);
      break;
    case 44:
      TIMSK5 = (0 << OCIE5C);
      break;
    case 45:
      TIMSK5 = (0 << OCIE5B);
      break;
    case 46:
      TIMSK5 = (0 << OCIE5A);   
      break;
    default:
      error = 1;
      break;
  }// end switch

  return error;
}



/*
// Interrupt Service Routines

ISR(TIMER1_COMPA_vect){ PWMHandler::ISR_OC1A(); }
ISR(TIMER1_COMPB_vect){ pfi_OC1B(); }
ISR(TIMER1_COMPC_vect){ pfi_OC1C(); }

ISR(TIMER3_COMPA_vect){ pfi_OC3A(); }
ISR(TIMER3_COMPB_vect){ pfi_OC3B(); }
ISR(TIMER3_COMPC_vect){ pfi_OC3C(); }

ISR(TIMER4_COMPA_vect){ pfi_OC4A(); }
ISR(TIMER4_COMPB_vect){ pfi_OC4B(); }
ISR(TIMER4_COMPC_vect){ pfi_OC4C(); }

ISR(TIMER5_COMPA_vect){ pfi_OC5A(); }
ISR(TIMER5_COMPB_vect){ pfi_OC5B(); }
ISR(TIMER5_COMPC_vect){ pfi_OC5C(); }
*/
