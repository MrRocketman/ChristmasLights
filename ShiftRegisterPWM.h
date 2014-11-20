/*
ShiftRegisterPWM.h - Library for Arduino to PWM many outputs using shift registers
Copyright (c) 2011-2012 Elco Jacobs, www.elcojacobs.com
All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef ShiftRegisterPWM_H
#define ShiftRegisterPWM_H

#include "pins_arduino_compile_time.h" // My own version of pins arduino, which does not define the arrays in program memory
#include <Arduino.h>
#include "CShiftRegisterPWM.h"


// These should be defined in the file where ShiftRegisterPWM.h is included.
extern const int ShiftRegisterPWM_latchPin;
extern const bool ShiftRegisterPWM_invertOutputs;
extern const bool ShiftRegisterPWM_balanceLoad;

// The ShiftRegisterPWM object is created in the header file, instead of defining it as extern here and creating it in the cpp file.
// If the ShiftRegisterPWM object is created in the cpp file, it is separately compiled with the library.
// The compiler cannot treat it as constant and cannot optimize well: it will generate many memory accesses in the interrupt function.

#if defined(ShiftRegisterPWM_USE_TIMER2)
	#if !defined(OCR2A)
		#error "The avr you are using does not have a timer2"
	#endif
#elif defined(ShiftRegisterPWM_USE_TIMER3)
	#if !defined(OCR3A)
		#error "The avr you are using does not have a timer3"
	#endif
#endif


#ifndef ShiftRegisterPWM_NOSPI
	// Use SPI
	#if defined(ShiftRegisterPWM_USE_TIMER3)
		CShiftRegisterPWM ShiftRegisterPWM(3,false,ShiftRegisterPWM_latchPin,MOSI,SCK);
	#elif defined(ShiftRegisterPWM_USE_TIMER2)
		CShiftRegisterPWM ShiftRegisterPWM(2,false,ShiftRegisterPWM_latchPin,MOSI,SCK);
	#else
		CShiftRegisterPWM ShiftRegisterPWM(1,false,ShiftRegisterPWM_latchPin,MOSI,SCK);
	#endif
#else
	// Don't use SPI
	extern const int ShiftRegisterPWM_clockPin;
	extern const int ShiftRegisterPWM_dataPin;
	#if defined(ShiftRegisterPWM_USE_TIMER3)
		CShiftRegisterPWM ShiftRegisterPWM(3,true,ShiftRegisterPWM_latchPin,ShiftRegisterPWM_dataPin,ShiftRegisterPWM_clockPin);
	#elif defined(ShiftRegisterPWM_USE_TIMER2)
		CShiftRegisterPWM ShiftRegisterPWM(2,true,ShiftRegisterPWM_latchPin,ShiftRegisterPWM_dataPin,ShiftRegisterPWM_clockPin);
	#else
		CShiftRegisterPWM ShiftRegisterPWM(1,true,ShiftRegisterPWM_latchPin,ShiftRegisterPWM_dataPin,ShiftRegisterPWM_clockPin);
	#endif
#endif

// The macro below uses 3 instructions per pin to generate the byte to transfer with SPI
// Retreive duty cycle setting from memory (ldd, 2 clockcycles)
// Compare with the counter (cp, 1 clockcycle) --> result is stored in carry
// Use the rotate over carry right to shift the compare result into the byte. (1 clockcycle).
#define add_one_pin_to_byte(sendbyte, counter, ledPtr) \
{ \
	unsigned char pwmval=*ledPtr; \
	asm volatile ("cp %0, %1" : /* No outputs */ : "r" (counter), "r" (pwmval): ); \
	asm volatile ("ror %0" : "+r" (sendbyte) : "r" (sendbyte) : ); 	\
}

// The inline function below uses normal output pins to send one bit to the SPI port.
// This function is used in the noSPI mode and is useful if you need the SPI port for something else.
// It is a lot 2.5x slower than the SPI version.
static inline void pwm_output_one_pin(volatile uint8_t * const clockPort, volatile uint8_t * const dataPort,\
                                  const uint8_t clockBit, const uint8_t dataBit, \
                                  unsigned char counter, unsigned char * ledPtr){
    bitClear(*clockPort, clockBit);
    if(ShiftRegisterPWM_invertOutputs){
      bitWrite(*dataPort, dataBit, *(ledPtr)<=counter );
    }
    else{
      bitWrite(*dataPort, dataBit, *(ledPtr)>counter );
    }
    bitSet(*clockPort, clockBit);
}

static inline void ShiftRegisterPWM_handleInterrupt(void){
	sei(); //enable interrupt nesting to prevent disturbing other interrupt functions (servo's for example).

	// Look up which bit of which output register corresponds to the pin.
	// This should be constant, so the compiler can optimize this code away and use sbi and cbi instructions
	// The compiler only knows this if this function is compiled in the same file as the pin setting.
	// That is the reason the full funcion is in the header file, instead of only the prototype.
	// If this function is defined in cpp files of the library, it is compiled seperately from the main file.
	// The compiler does not recognize the pins/ports as constant and sbi and cbi instructions cannot be used.


	volatile uint8_t * const latchPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[ShiftRegisterPWM_latchPin]];
	const uint8_t latchBit =  digital_pin_to_bit_PGM_ct[ShiftRegisterPWM_latchPin];

	#ifdef ShiftRegisterPWM_NOSPI
	volatile uint8_t * const clockPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[ShiftRegisterPWM_clockPin]];
	volatile uint8_t * const dataPort  = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[ShiftRegisterPWM_dataPin]];
	const uint8_t clockBit =  digital_pin_to_bit_PGM_ct[ShiftRegisterPWM_clockPin];
	const uint8_t dataBit =   digital_pin_to_bit_PGM_ct[ShiftRegisterPWM_dataPin];   
	#endif

	// Define a pointer that will be used to access the values for each output. 
	// Let it point one past the last value, because it is decreased before it is used.

	unsigned char * ledPtr=&ShiftRegisterPWM.m_PWMValues[ShiftRegisterPWM.m_amountOfOutputs];

	// Write shift register latch clock low 
	bitClear(*latchPort, latchBit);
	unsigned char counter = ShiftRegisterPWM.m_counter;
	
	#ifndef ShiftRegisterPWM_NOSPI
	//Use SPI to send out all bits
	SPDR = 0; // write bogus bit to the SPI, because in the loop there is a receive before send.
	for(unsigned char i = ShiftRegisterPWM.m_amountOfRegisters; i>0;--i){   // do a whole shift register at once. This unrolls the loop for extra speed
		unsigned char sendbyte;  // no need to initialize, all bits are replaced
		if(ShiftRegisterPWM_balanceLoad){
			counter +=8; // distribute the load by using a shifted counter per shift register
		}
		add_one_pin_to_byte(sendbyte, counter, --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);

		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);
		add_one_pin_to_byte(sendbyte, counter,  --ledPtr);

		while (!(SPSR & _BV(SPIF)));    // wait for last send to finish and retreive answer. Retreive must be done, otherwise the SPI will not work.
		if(ShiftRegisterPWM_invertOutputs){	
			sendbyte = ~sendbyte; // Invert the byte if needed.
		}
		SPDR = sendbyte; // Send the byte to the SPI
	}
	while (!(SPSR & _BV(SPIF))); // wait for last send to complete.
	#else
	//Use port manipulation to send out all bits
	for(unsigned char i = ShiftRegisterPWM.m_amountOfRegisters; i>0;--i){   // do one shift register at a time. This unrolls the loop for extra speed
		if(ShiftRegisterPWM_balanceLoad){
			counter +=8; // distribute the load by using a shifted counter per shift register
		}
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);  // This takes 12 or 13 clockcycles
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
		pwm_output_one_pin(clockPort, dataPort, clockBit, dataBit, counter, --ledPtr);
	}
	#endif

	// Write shift register latch clock high
	bitSet(*latchPort, latchBit);

	if(ShiftRegisterPWM.m_counter<ShiftRegisterPWM.m_maxBrightness){
		ShiftRegisterPWM.m_counter++; // Increase the counter
	}
	else{
		ShiftRegisterPWM.m_counter=0; // Reset counter if it maximum brightness has been reached
	}
}

// See table  11-1 for the interrupt vectors */
#if defined(ShiftRegisterPWM_USE_TIMER3)
	//Install the Interrupt Service Routine (ISR) for Timer3 compare and match A.
	ISR(TIMER3_COMPA_vect) {
		ShiftRegisterPWM_handleInterrupt();
	}
#elif defined(ShiftRegisterPWM_USE_TIMER2)
	//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
	ISR(TIMER2_COMPA_vect) {
		ShiftRegisterPWM_handleInterrupt();
	}
#else
	//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
	ISR(TIMER1_COMPA_vect) {
		ShiftRegisterPWM_handleInterrupt();
	}
#endif

// #endif for include once.
#endif
