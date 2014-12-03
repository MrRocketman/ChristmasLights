// ******** Commands ************ //
// BoardID  Command     Data                                                            End of Packet Byte      Description
//
// 0x01     0x01        0x45(ch #)                                                                  0xFF        1 Channel on
// 0x01     0x02        0x8A(ch #)                                                                  0xFF        1 Channel off
//
// 0x01     0x04        0x05(0b00000101 = channel 0, 2 on; All others off) 0x01(channel 8 on) ....  0xFF        N Channel state
//
// 0x01     0x10        0x10(Ch #) 0x5F(Dim)                                                        0xFF        Set Brightness for 1 channel
// 0x01     0x11        0x5F(Dim)                                                                   0xFF        Set Brightness for all channels
//
// 0x01     0x15                                                                                    0xFF        All channels on
// 0x01     0x16                                                                                    0xFF        All channels off
//
// 0x01     0x20        0x10(Ch #) 0xEE(Time - 0.01 sec per bit)                                    0xFF        Fade channel up over time (hundreths)
// 0x01     0x21        0x10(Ch #) 0xEE(Time - 0.1 sec per bit)                                     0xFF        Fade channel up over time (tenths)
// 0x01     0x22        0x10(Ch #) 0xEE(Time - 0.01 sec per bit)                                    0xFF        Fade channel down over time (hundreths)
// 0x01     0x23        0x10(Ch #) 0xEE(Time - 0.1 sec per bit)                                     0xFF        Fade channel down over time (tenths)
// 0x01     0x24        0x10(Ch #) 0xF0(Start Dim) 0xFF(End Dim) 0xEE(Time - 0.01 sec per bit)      0xFF        Fade channel from x to y over time (hundreths)
// 0x01     0x25        0x10(Ch #) 0xF0(Start Dim) 0xFF(End Dim) 0xEE(Time - 0.1 sec per bit)       0xFF        Fade channel from x to y over time (tenths)
//
// 0x01     0x30        0xEE(Time - 0.01 sec per bit)                                               0xFF        Fade all up over time (hundreths)
// 0x01     0x31        0xEE(Time - 0.1 sec per bit)                                                0xFF        Fade all up over time (tenths)
// 0x01     0x32        0xEE(Time - 0.01 sec per bit)                                               0xFF        Fade all down over time (hundreths)
// 0x01     0x33        0xEE(Time - 0.1 sec per bit)                                                0xFF        Fade all down over time (tenths)
// 0x01     0x34        0xF0(Start Dim) 0xFF(End Dim) 0xEE(Time - 0.01 sec per bit)                 0xFF        Fade all from x to y over time (hundreths)
// 0x01     0x35        0xF0(Start Dim) 0xFF(End Dim) 0xEE(Time - 0.1 sec per bit)                  0xFF        Fade all from x to y over time (tenths)
//
//
// 0x01     0xF1                                                                                    0xFF        Rquest status (number of boards)







// Mad props to Elco Jacobs for his ShiftPWM library which I had modified and incorporated directly into the code rather than using it as a library. See his ShiftPWM for awesome comments on everything. I've stripped lots of them out

/*
 ShiftPWM.h - Library for Arduino to PWM many outputs using shift registers
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

/******************************************************************************
 * This example shows how to use the ShiftPWM library to PWM many outputs.
 * All shift registers are chained, so they can be driven with 3 pins from the arduino.
 * A timer interrupt updates all PWM outputs according to their duty cycle setting.
 * The outputs can be inverted by making ShiftPWM_invertOutputs true.
 *
 * How the library works:
 * The ShiftPWM class keeps a setting for the duty cycle for each output pin, which
 * can be set using the provided functions. It also keeps a counter which it compares
 * to these duty cycles. This timer continuously runs from 0 to the maximum duty cycle.
 *
 * A timer interrupt is configured by ShiftPWM.Start(pwmFrequency,maxBrightness).
 * The interrupt frequency is set to pwmFrequency * (maxBrightness+1).
 * Each interrupt all duty cycles are compared to the counter and the corresponding pin
 * is written 1 or 0 based on the result. Then the counter is increased by one.
 *
 * The duration of the interrupt depends on the number of shift registers (N).
 * T = 112 + 43*N
 *
 * The load of the interrupt function on your program can be calculated:
 * L = Interrupt frequency * interrupt duration / clock frequency
 * L = F*(Bmax+1)*(112+43*N)/F_CPU
 * Quick reference for load:
 * 3 registers  255 maxBrightness 75Hz  load = 0.29
 * 6 registers  255 maxBrightness 75Hz  load = 0.45
 * 24 registers 100 maxBrightness 75Hz  load = 0.54
 * 48 registers  64 maxBrightness 75Hz  load = 0.66
 * 96 registers  32 maxBrightness 75Hz  load = 0.66
 *
 * A higher interrupt load will mean less computional power for your main program,
 * so try to keep it as low as possible and at least below 0.9.
 *
 * The following functions are used:
 *
 * ShiftPWM.Start(int ledFrequency, int max_Brightness)		Enable ShiftPWM with desired frequency and brightness levels
 * ShiftPWM.SetAmountOfRegisters(int newAmount)			Set or change the amount of output registers. Can be changed at runtime.
 * ShiftPWM.PrintInterruptLoad()				Print information on timer usage, frequencies and interrupt load
 * ShiftPWM.OneByOneSlow()  				        Fade in and fade out all outputs slowly
 * ShiftPWM.OneByOneFast()					Fade in and fade out all outputs fast
 * ShiftPWM.SetOne(int pin, unsigned char value)		Set the duty cycle of one output
 * ShiftPWM.SetAll(unsigned char value)				Set all outputs to the same duty cycle
 *
 * ShiftPWM.SetGroupOf2(int group, unsigned char v0, unsigned char v1);
 * ShiftPWM.SetGroupOf3(int group, unsigned char v0, unsigned char v1, unsigned char v2);
 * ShiftPWM.SetGroupOf4(int group, unsigned char v0, unsigned char v1, unsigned char v2, unsigned char v3);
 * ShiftPWM.SetGroupOf5(int group, unsigned char v0, unsigned char v1, unsigned char v2, unsigned char v3, unsigned char v4);
 * 		--> Set a group of outputs to the given values. SetGroupOf3 is useful for RGB LED's. Each LED will be a group.
 *
 * Debug information for wrong input to functions is also send to the serial port,
 * so check the serial port when you run into problems.
 *
 * (c) Elco Jacobs, E-atelier Industrial Design TU/e, July 2011.
 *
 *****************************************************************************/
