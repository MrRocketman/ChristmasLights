#ifndef Pins_Arduino_Compile_Time_h
#define Pins_Arduino_Compile_time_h


/* This is an alternative to pins_arduino.h
In pins_arduino.h and .cpp the look up arrays are defined as:
const uint16_t PROGMEM port_to_output_PGM[]
This places the array in program memory. pgm_read_byte() functions are used to retreive the array from program memory at runtime.

Because the arrays are received at runtime, it takes some instructions to look up the outputs.
This is not a problem in most cases, but it is if you are trying to write super fast code.

To make use of efficient sbi (Set Bit Immidiate) and cbi (Clear Bit Immidiate), the pin to write to must be defined as const.
The compiler does not understand that digitalPinToPort(2) is constant and does not optimize it away.

In this file I redefine the arrays as:
volatile uint8_t * const port_to_output_PGM_ct[]

So it is a constant pointer to a volatile uint8_t: 
a pointer to an output register that stays in the same place (* const), but can change in value (volatile uint8_t)

These definitions are understood by the compiler and result in superfast code,
but still allow you to use the arduino pin numbers instead of registers like PORTB

(C) 2011-2012 Elco Jacobs. www.elcojacobs.com

*/

#include <pins_arduino.h>

#define NOT_A_PIN 0
#define NOT_A_PORT 0


// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
// (PWM+ indicates the additional PWM pins on the ATmega168.)

// ARDUINO
//
// 0-7 PE0-PE7   works
// 8-13 PB0-PB5  works
// 14-21 PA0-PA7 works
// 22-29 PH0-PH7 works
// 30-35 PG5-PG0 works
// 36-43 PC7-PC0 works
// 44-51 PJ7-PJ0 works
// 52-59 PL7-PL0 works
// 60-67 PD7-PD0 works
// A0-A7 PF0-PF7
// A8-A15 PK0-PK7

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)

volatile uint8_t *port_to_output_PGM_ct[] =
{
	NOT_A_PORT,
	NOT_A_PORT,
	&PORTB,
	&PORTC,
	&PORTD,
};


const uint8_t digital_pin_to_port_PGM_ct[] =
{
	PD, // 0 
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PD,
	PB, // 8 
	PB,
	PB,
	PB,
	PB,
	PB,
	PC, // 14 
	PC,
	PC,
	PC,
	PC,
	PC,
};

const uint8_t digital_pin_to_bit_PGM_ct[] =
{
	0, // 0, port D 
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	0, // 8, port B 
	1,
	2,
	3,
	4,
	5,
	0, // 14, port C /
	1,
	2,
	3,
	4,
	5,
};

#endif
