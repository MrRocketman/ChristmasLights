#include "pins_arduino_compile_time.h"
#include <SPI.h>
#include <Arduino.h>

#pragma mark - Variable Declarations

// Debugging defines
#define SERIAL_PRINTING // This disables most all serial printing
#ifdef SERIAL_PRINTING
    //#define DEBUG
#endif

// Board specific variables! Change these per board!
const byte boardID = 0x01;

////////////////////////////////////////////////////////////////////////////////////////////
// Don't change any variables below here unless you really, really know what you are doing//
////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_PACKET_SIZE 64

#define MICROSECONDS_TO_MILLISECONDS 1 / 1000.0
#define MILLISECONDS_TO_SECONDS 1 / 1000.0

// Phase Frequency Control (PFC) (Zero Cross)
unsigned long int previousZeroCrossTime = 0; // Timestamp in micros() of the latest zero crossing interrupt
unsigned long int currentZeroCrossTime = 0; // Timestamp in micros() of the previous zero crossing interrupt
unsigned long int zeroCrossTimeDifference =  0; // The calculated micros() between the last two zero crossings
volatile byte zeroCrossTrigger = 0;
byte zeroCrossPin = 2;

// BoardDetectValues
byte boardDetectPin = A5;
short boardDetectValue = 0;
short oldBoardDetectValue = 0;

// Packet Variables
const byte endOfPacketByte = 0xFF; // ASCII value 255 is our end of command byte
byte packetBuffer[MAX_PACKET_SIZE]; // buffer for serial data
byte packetBufferLength = 0;
byte currentByteFromPacket = 0;
byte currentByteIndex = 0;

// Shift PWM Pins
const int latchPin = 10;
volatile uint8_t *const latchPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[latchPin]];
const uint8_t latchBit =  digital_pin_to_bit_PGM_ct[latchPin];
volatile uint8_t * const clockPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[SCK]];
volatile uint8_t * const dataPort  = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[MOSI]];
const uint8_t clockBit = digital_pin_to_bit_PGM_ct[SCK];
const uint8_t dataBit = digital_pin_to_bit_PGM_ct[MOSI];

// Shift Register
const int timerToUse = 1; // Can also be timer 2 or timer 3. Timer1 is the best, timer3 works on Leonardo or Mega, timer2 is the worst.
int prescaler;
float pwmFrequency = 120;
byte maxBrightness = 254;
byte numberOfShiftRegisters = 0;
byte numberOfChannels = 0;
byte *pwmValues = 0;
volatile byte brightnessCounter = maxBrightness;

// workaround for a bug in WString.h
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

#if (timerToUse == 2)
    #if !defined(OCR2A)
        #error "The avr you are using does not have a timer2"
    #endif
#elif (timerToUse == 3)
    #if !defined(OCR3A)
        #error "The avr you are using does not have a timer3"
    #endif
#endif

// The macro below uses 3 instructions per pin to generate the byte to transfer with SPI. Retreive duty cycle setting from memory (ldd, 2 clockcycles). Compare with the counter (cp, 1 clockcycle) --> result is stored in carry. Use the rotate over carry right to shift the compare result into the byte. (1 clockcycle).
// TL;DR super fast way of doing the following
// if(counter < pwmVal)
//      turnOn;
// else
//      turnOff;
#define add_one_pin_to_byte(sendbyte, counter, ledPtr) \
{ \
unsigned char pwmval=*ledPtr; \
asm volatile ("cp %0, %1" : /* No outputs */ : "r" (counter), "r" (pwmval): ); \
asm volatile ("ror %0" : "+r" (sendbyte) : "r" (sendbyte) : ); 	\
}

#pragma mark - Method Declarations

// Packet Processing
void processPacket();
void readNextByteInPacket();
void clearPacketBuffer();

// State changing methods
void turnOnChannel(byte channelNumber);
void turnOffChannel(byte channelNumber);
void setBrightnessForChannel(byte channelNumber, byte brightness);
bool isChannelNumberValid(byte channelNumber);

// Board Detections
void boardDetect();
byte boardDetectValueIsNearValueWithRange(int compareValue = 512, int range = 10);

// Zero Cross
void zeroCrossDetect();

// ShiftRegister
void handleInterrupt();
void updateInterruptClock();

void initTimer1();
#if defined(OCR3A)
// Arduino Leonardo or Micro (32u4)
void initTimer3();
#endif
#if defined(OCR2A)
// Normal Arduino (328)
void initTimer2();
#endif
void printInterruptLoad();

void oneByOneSlow();
void oneByOneFast();
void oneByOneCore(int delaytime);

#pragma mark - Setup And Main Loop

// free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
int availableMemory()
{
    // Use 1024 with ATmega168
    int size = 2048;
    byte *buf;
    while ((buf = (byte *) malloc(--size)) == NULL);
    free(buf);
    return size;
}

void setup()
{
    // Setup our pins
    pinMode(latchPin, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(SS, OUTPUT); // Warning: if the SS pin ever becomes a LOW INPUT then SPI automatically switches to Slave, so the data direction of the SS pin MUST be kept as OUTPUT.
    pinMode(boardDetectPin, INPUT);
    pinMode(zeroCrossPin, INPUT);
    
    digitalWrite(SCK, LOW);
    digitalWrite(MOSI, LOW);
    digitalWrite(SS, HIGH);
    
    // Serial setup
    Serial.begin(57600); // Xbee doesn't like to run any faster than 57600 for sustained throughput!
    
    // SPI Setup
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    // Set clock polarity and phase for shift registers (Mode 3)
    SPCR |= _BV(CPOL);
    SPCR |= _BV(CPHA);
    SPCR |= _BV(MSTR);
    SPCR |= _BV(SPE);
    SPI.begin();
    
    if(isInterruptLoadAcceptable())
    {
        if(timerToUse == 1)
        {
            Serial.println("Timer1");
            initTimer1();
        }
#if defined(USBCON)
        else if(timerToUse == 3)
        {
            Serial.println("Timer3");
            initTimer3();
        }
#else
        else if(timerToUse == 2)
        {
            Serial.println("Timer2");
            initTimer2();
        }
#endif
    }
    else
    {
        Serial.println(F("Interrupts are disabled because load is too high."));
        cli(); //Disable interrupts
    }
    
    // Setup the zero cross interrupt
    attachInterrupt(0, zeroCrossDetect, FALLING); // This uses zeroCrossPin
    
#ifdef SERIAL_PRINTING
    Serial.print("LD:");
    Serial.print(boardID);
    Serial.println(",Online");
#endif
}

void loop()
{
    // Detect MOSFET/TRIAC boards
    if(numberOfShiftRegisters == 0)
    {
        boardDetect();
    }
    
    // Handle AC Zero Cross
    if(zeroCrossTrigger)
    {
        handleZeroCross();
        zeroCrossTrigger = 0;
    }
    
    if(digitalRead(zeroCrossPin) == 0)
    {
        Serial.println(micros());
    }
    
    // Handle XBee data
	if(Serial.available())
    {
        packetBuffer[packetBufferLength] = Serial.read();
        packetBufferLength ++;
        
        // End of command byte, so now we'll proccess the command
        if(packetBuffer[packetBufferLength - 1] == endOfPacketByte)
        {
            processPacket();
            clearPacketBuffer();
        }
        // Update everything so the next byte can be received
        else
        {
            // Somehow the buffer has overflowed. Erase it since it's probably just garbage
            if(packetBufferLength > MAX_PACKET_SIZE)
            {
#ifdef SERIAL_PRINTING
                Serial.print("LD:");
                Serial.print(boardID);
                Serial.println(",Buffer overflow");
#endif
                
                clearPacketBuffer();
            }
        }
    }
}

#pragma mark - Command Processing

void processPacket()
{
#ifdef DEBUG
    for(byte i = 0; i < packetBufferLength; i ++)
    {
        Serial.print(packetBuffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif
    
    readNextByteInPacket();
    
    // Only process the packet if it's for this board
    if(currentByteFromPacket == boardID)
    {
        readNextByteInPacket();
        
        if(currentByteFromPacket == 0x01) // Command 0x01 (1 channel on)
        {
            readNextByteInPacket();
            
            turnOnChannel(currentByteFromPacket);
        }
        else if(currentByteFromPacket == 0x02) // Command 0x02 (1 Channel off)
        {
            readNextByteInPacket();
            
            turnOffChannel(currentByteFromPacket);
        }
        else if(currentByteFromPacket == 0x03) // Command 0x03 (All on)
        {
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                turnOnChannel(i);
            }
        }
        else if(currentByteFromPacket == 0x04) // Command 0x04 (All off)
        {
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                turnOffChannel(i);
            }
        }
        else if(currentByteFromPacket == 0x05) // Command 0x05 (1 channel brightness)
        {
            readNextByteInPacket();
            byte channelNumber = currentByteFromPacket;
            
            readNextByteInPacket();
            
            setBrightnessForChannel(channelNumber, currentByteFromPacket);
        }
        else if(currentByteFromPacket == 0x06) // Command 0x06 (All brightness)
        {
            readNextByteInPacket();
            
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                setBrightnessForChannel(i, currentByteFromPacket);
            }
        }
        else if(currentByteFromPacket == 0x07) // Command 0x07 (Fade Channel over time in hundreths of seconds)
        {
            /*byte channelNumber;
            byte startBrightness;
            byte endBrightness;
            byte fadeTimeInHundrethsOfSeconds;
            byte brightnessChange;
            byte brightnessChangeIsPositive;
            unsigned int totalMillisecondsForFade;
            
            // Get the channel number
            readNextByteInPacket();
            channelNumber = currentByteFromPacket;
            
            // Get the start and end brightness (each are only 4 bit values)
            readNextByteInPacket();
            startBrightness = (currentByteFromPacket & 0x0F); // Get the first 4 bits
            endBrightness = (currentByteFromPacket & 0xF0); // Get the other 4 bits
            
            // Get the fade time
            readNextByteInPacket();
            fadeTimeInHundrethsOfSeconds = currentByteFromPacket;
            
            // Calculate the fade information
            totalMillisecondsForFade = fadeTimeInHundrethsOfSeconds * 10;
            brightnessChange = abs(endBrightness - startBrightness);
            brightnessChangeIsPositive = (endBrightness > startBrightness ? 1 : 0);
            
            //millisecondBeforeUpdatingChannel[channelNumber] =*/
        }
        else if(currentByteFromPacket == 0xF1) // Command 0xF1 (Request status - number of boards connected)
        {
            Serial.println(numberOfShiftRegisters);
        }
    }
}

void readNextByteInPacket()
{
    if(currentByteIndex < packetBufferLength)
    {
        currentByteFromPacket = packetBuffer[currentByteIndex];
    }
    else
    {
        currentByteFromPacket = 0;
    }
    
    currentByteIndex ++;
}

void clearPacketBuffer()
{
	packetBufferLength = 0;
    currentByteIndex = 0;
	
	memset(packetBuffer, 0, MAX_PACKET_SIZE);
}

#pragma mark - Channel Changes

void turnOnChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, maxBrightness);
}

void turnOffChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, 0);
}

void setBrightnessForChannel(byte channelNumber, byte brightness)
{
    // Set the brightness
    if(isChannelNumberValid(channelNumber))
    {
        pwmValues[channelNumber] = brightness;
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" v:");
    Serial.println(brightness);
#endif
}

bool isChannelNumberValid(byte channelNumber)
{
    if(channelNumber < numberOfChannels)
    {
        return 1;
    }
    else
    {
        Serial.print(F("Error: Trying to change a channel that isn't initialized"));
        return 0;
    }
}

#pragma mark - Shift Register/Board Detection

void boardDetect()
{
    oldBoardDetectValue = boardDetectValue;
    boardDetectValue = analogRead(boardDetectPin);
    
    // See if there has been a change in the boardDetect Value
    if(abs(boardDetectValue - oldBoardDetectValue) > 7)
    {
        if(boardDetectValueIsNearValueWithRange(0))
        {
            numberOfShiftRegisters = 0;
            //Serial.println("NO BOARDS DETECTED! CHECK THE JUMPER!");
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 2))
        {
            numberOfShiftRegisters = 1;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 3))
        {
            numberOfShiftRegisters = 2;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 4))
        {
            numberOfShiftRegisters = 3;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 5))
        {
            numberOfShiftRegisters = 4;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 6))
        {
            numberOfShiftRegisters = 5;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 7))
        {
            numberOfShiftRegisters = 6;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 8))
        {
            numberOfShiftRegisters = 7;
        }
        else if(boardDetectValueIsNearValueWithRange(1024 / 9))
        {
            numberOfShiftRegisters = 8;
        }
        
        cli(); // Disable interrupt
        numberOfChannels = numberOfShiftRegisters * 8;
        
        // Check if new amount will not result in deadlock
        if(isInterruptLoadAcceptable())
        {
            // Resize pwmValues array
            pwmValues = (byte *)realloc(pwmValues, numberOfChannels);
            // Initialize all pwmValues to 0
            memset(pwmValues, 0, numberOfChannels);
            // Re-enable interrupt
            sei();
        }
        else
        {
            // New value would result in deadlock, keep old values and print an error message
            Serial.println(F("Number of shift registers not changed, because load would become too high"));
            // Re-enable interrupt
            sei();
        }
        
        setBrightnessForChannel(0, 50);
        setBrightnessForChannel(1, 128);
        
        #ifdef DEBUG
            Serial.print("bd:");
            Serial.println(boardDetectValue);
        #endif
        
#ifdef DEBUG
        // Print Shift PWM stuff
        printInterruptLoad();
        
        // Print Available RAM
        Serial.print("RAM:");
        Serial.print(availableMemory());
        Serial.println(" Bytes");
#endif
    }
}

byte boardDetectValueIsNearValueWithRange(int compareValue, int range)
{
    if(boardDetectValue >= compareValue - range && boardDetectValue <= compareValue + range)
    {
        return 1;
    }
    else if(compareValue < range && boardDetectValue < range)
    {
        return 1;
    }
    else if(compareValue > 1024 - range && boardDetectValue > 1024 - range)
    {
        return 1;
    }
    
    return 0;
}

#pragma mark - Zero Cross

// Hardware zero cross interrupt
void zeroCrossDetect()
{
    zeroCrossTrigger = 1;
}

// Handle the zero cross
void handleZeroCross()
{
    // Calculate the frequency
    previousZeroCrossTime = currentZeroCrossTime;
    currentZeroCrossTime = micros();
    zeroCrossTimeDifference = currentZeroCrossTime - previousZeroCrossTime;
    pwmFrequency = 1.0 / (zeroCrossTimeDifference * MICROSECONDS_TO_MILLISECONDS * MILLISECONDS_TO_SECONDS); // Divide by 2 to get 60
    updateInterruptClock();
}

#pragma mark - Shift Register

void updateInterruptClock()
{
    // Reset the brightnessCounter
    brightnessCounter = maxBrightness;
    //Serial.print("z:");
    ///Serial.println(micros());
    
    if(timerToUse == 1)
    {
        OCR1A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
    }
#if defined(USBCON)
    else if(timerToUse == 3)
    {
        OCR3A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
    }
#else
    else if(timerToUse == 2)
    {
        OCR2A = round(((float)F_CPU / (float)prescaler) / (pwmFrequency * ((float)maxBrightness + 1)) - 1);
    }
#endif
}

// See table  11-1 for the interrupt vectors
#if (timerToUse == 3)
//Install the Interrupt Service Routine (ISR) for Timer3 compare and match A.
ISR(TIMER3_COMPA_vect) {
    handleInterrupt();
}
#elif (timerToUse == 2)
//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
ISR(TIMER2_COMPA_vect) {
    handleInterrupt();
}
#else
//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
ISR(TIMER1_COMPA_vect) {
    handleInterrupt();
}
#endif

void handleInterrupt()
{
    //sei(); //enable interrupt nesting to prevent disturbing other interrupt functions (servo's for example).
    
    // Define a pointer that will be used to access the values for each output.
    // Let it point one past the last value, because it is decreased before it is used.
    unsigned char *ledPtr = &pwmValues[numberOfChannels];
    
    // Write shift register latch clock low
    bitClear(*latchPort, latchBit);
    
    // Write bogus bit to the SPI, because in the loop there is a receive before send.
    SPDR = 0;
    // Do a whole shift register at once. This unrolls the loop for extra speed
    for(unsigned char i = numberOfShiftRegisters; i > 0; --i)
    {
        unsigned char sendbyte;  // no need to initialize, all bits are replaced
        
        add_one_pin_to_byte(sendbyte, brightnessCounter, --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        add_one_pin_to_byte(sendbyte, brightnessCounter,  --ledPtr);
        
        // wait for last send to finish and retreive answer. Retreive must be done, otherwise the SPI will not work.
        while (!(SPSR & _BV(SPIF)));
        
        SPDR = sendbyte; // Send the byte to the SPI
    }
    // wait for the send to complete.
    while (!(SPSR & _BV(SPIF)));
    
    // Write shift register latch clock high
    bitSet(*latchPort, latchBit);
    
    if(brightnessCounter > 0)
    {
        // Decrease the counter. This is the key to getting AC dimming since triacs stay on until the next zero cross. So we need to turn on after a delay rather than turn on immediately and turn off after a delay
        brightnessCounter --;
    }
    /*else
    {
        // Reset counter if it maximum brightness has been reached
        brightnessCounter = 0;
        Serial.print("i:");
        Serial.println(micros());
    }*/
}

bool isInterruptLoadAcceptable()
{
    // This function calculates if the interrupt load would become higher than 0.9 and prints an error if it would.
    // This is with inverted outputs, which is worst case. Without inverting, it would be 42 per register.
    float interruptDuration;
    interruptDuration = 96 + 108 * (float)numberOfShiftRegisters;
    float interruptFrequency = pwmFrequency * ((float)maxBrightness + 1);
    float load = interruptDuration * interruptFrequency / F_CPU;
    
    if(load > 0.9)
    {
        Serial.print(F("New interrupt load would be "));
        Serial.print(load);
        Serial.println(F(" , which is too high."));
        return 0;
    }
    else
    {
        return 1;
    }
}

void initTimer1()
{
    // Configure timer1 in CTC mode: clear the timer on compare match. See the Atmega328 Datasheet 15.9.2 for an explanation on CTC mode. See table 15-4 in the datasheet.
    bitSet(TCCR1B, WGM12);
    bitClear(TCCR1B, WGM13);
    bitClear(TCCR1A, WGM11);
    bitClear(TCCR1A, WGM10);
    
    // Select clock source: internal I/O clock, without a prescaler. This is the fastest possible clock source for the highest accuracy. See table 15-5 in the datasheet.
    bitSet(TCCR1B, CS10);
    bitClear(TCCR1B, CS11);
    bitClear(TCCR1B, CS12);
    
    // The timer will generate an interrupt when the value we load in OCR1A matches the timer value. One period of the timer, from 0 to OCR1A will therefore be (OCR1A+1)/(timer clock frequency). We want the frequency of the timer to be (LED frequency)*(number of brightness levels). So the value we want for OCR1A is: timer clock frequency/(LED frequency * number of bightness levels)-1
    prescaler = 1;
    OCR1A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
    
    // Enable the timer interrupt, see datasheet  15.11.8)
    bitSet(TIMSK1,OCIE1A);
}

#if defined(OCR2A)
void initTimer2()
{
    // Configure timer2 in CTC mode: clear the timer on compare match. See the Atmega328 Datasheet 15.9.2 for an explanation on CTC mode. See table 17-8 in the datasheet.
    bitClear(TCCR2B, WGM22);
    bitSet(TCCR2A, WGM21);
    bitClear(TCCR2A, WGM20);
    
    // Select clock source: internal I/O clock, calculate most suitable prescaler. This is only an 8 bit timer, so choose the prescaler so that OCR2A fits in 8 bits. See table 15-5 in the datasheet.
    int compare_value = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1)) - 1);
    if(compare_value <= 255)
    {
        prescaler = 1;
        bitClear(TCCR2B, CS22);
        bitClear(TCCR2B, CS21);
        bitClear(TCCR2B, CS20);
    }
    else if(compare_value / 8 <= 255)
    {
        prescaler = 8;
        bitClear(TCCR2B, CS22);
        bitSet(TCCR2B, CS21);
        bitClear(TCCR2B, CS20);
    }
    else if(compare_value / 32 <= 255)
    {
        prescaler = 32;
        bitClear(TCCR2B, CS22);
        bitSet(TCCR2B, CS21);
        bitSet(TCCR2B, CS20);
    }
    else if(compare_value / 64 <= 255)
    {
        prescaler = 64;
        bitSet(TCCR2B, CS22);
        bitClear(TCCR2B, CS21);
        bitClear(TCCR2B, CS20);
    }
    else if(compare_value / 128 <= 255)
    {
        prescaler = 128;
        bitSet(TCCR2B, CS22);
        bitClear(TCCR2B, CS21);
        bitSet(TCCR2B, CS20);
    }
    else if(compare_value / 256 <= 255)
    {
        prescaler = 256;
        bitSet(TCCR2B, CS22);
        bitSet(TCCR2B, CS21);
        bitClear(TCCR2B, CS20);
    }
    
    // The timer will generate an interrupt when the value we load in OCR2A matches the timer value. One period of the timer, from 0 to OCR2A will therefore be (OCR2A+1)/(timer clock frequency). We want the frequency of the timer to be (LED frequency)*(number of brightness levels). So the value we want for OCR2A is: timer clock frequency/(LED frequency * number of bightness levels)-1
    OCR2A = round(((float)F_CPU / (float)prescaler) / (pwmFrequency * ((float)maxBrightness + 1)) - 1);
    // Enable the timer interrupt, see datasheet  15.11.8)
    bitSet(TIMSK2, OCIE2A);
}
#endif

#if defined(OCR3A)
// Arduino Leonardo or Micro
void initTimer3()
{
    // Only available on Leonardo and micro. Configure timer3 in CTC mode: clear the timer on compare match. See the Atmega32u4 Datasheet 15.10.2 for an explanation on CTC mode. See table 14-5 in the datasheet.
    bitSet(TCCR3B, WGM32);
    bitClear(TCCR3B, WGM33);
    bitClear(TCCR3A, WGM31);
    bitClear(TCCR3A, WGM30);
    
    // Select clock source: internal I/O clock, without a prescaler. This is the fastest possible clock source for the highest accuracy. See table 15-5 in the datasheet.
    bitSet(TCCR3B, CS30);
    bitClear(TCCR3B, CS31);
    bitClear(TCCR3B, CS32);
    
    // The timer will generate an interrupt when the value we load in OCR1A matches the timer value. One period of the timer, from 0 to OCR1A will therefore be (OCR1A+1)/(timer clock frequency). We want the frequency of the timer to be (LED frequency)*(number of brightness levels). So the value we want for OCR1A is: timer clock frequency/(LED frequency * number of bightness levels)-1
    prescaler = 1;
    OCR3A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
    // Enable the timer interrupt, see datasheet  15.11.8)
    bitSet(TIMSK3, OCIE3A);
}
#endif

void printInterruptLoad()
{
    //This function prints information on the interrupt settings for ShiftRegisterPWM. It runs a delay loop 2 times: once with interrupts enabled, once disabled. From the difference in duration, it can calculate the load of the interrupt on the program.
    unsigned long start1,end1,time1,start2,end2,time2,k;
    double load, cycles_per_int, interrupt_frequency;
    
    if(timerToUse == 1)
    {
        if(TIMSK1 & (1 << OCIE1A))
        {
            // interrupt is enabled, continue
        }
        else
        {
            // interrupt is disabled
            Serial.println(F("Interrupt is disabled."));
            return;
        }
    }
#if defined(USBCON)
    else if(timerToUse == 3)
    {
        if(TIMSK3 & (1 << OCIE3A))
        {
            // interrupt is enabled, continue
        }
        else
        {
            // interrupt is disabled
            Serial.println(F("Interrupt is disabled."));
            return;
        }
    }
#else
    else if(timerToUse == 2)
    {
        if(TIMSK2 & (1 << OCIE2A))
        {
            // interrupt is enabled, continue
        }
        else
        {
            // interrupt is disabled
            Serial.println(F("Interrupt is disabled."));
            return;
        }
    }
#endif
    
    //run with interrupt enabled
    start1 = micros();
    for(k = 0; k < 100000; k++)
    {
        delayMicroseconds(1);
    }
    end1 = micros();
    time1 = end1 - start1;
    
    //Disable Interrupt
    if(timerToUse == 1)
    {
        bitClear(TIMSK1, OCIE1A);
    }
#if defined(USBCON)
    else if(timerToUse == 3)
    {
        bitClear(TIMSK3, OCIE3A);
    }
#else
    else if(timerToUse == 2)
    {
        bitClear(TIMSK2, OCIE2A);
    }
#endif
    
    // run with interrupt disabled
    start2 = micros();
    for(k = 0; k < 100000; k++)
    {
        delayMicroseconds(1);
    }
    end2 = micros();
    time2 = end2 - start2;
    
    // ready for calculations
    load = (double)(time1 - time2) / (double)(time1);
    if(timerToUse == 1)
    {
        interrupt_frequency = (F_CPU / prescaler) / (OCR1A + 1);
    }
#if defined(USBCON)
    else if(timerToUse == 3)
    {
        interrupt_frequency = (F_CPU / prescaler) / (OCR3A + 1);
    }
#else
    else if(timerToUse == 2)
    {
        interrupt_frequency = (F_CPU / prescaler) / (OCR2A + 1);
    }
#endif
    cycles_per_int = load * (F_CPU / interrupt_frequency);
    
    //Ready to print information
    Serial.print(F("Load of interrupt: "));
    Serial.println(load,10);
    Serial.print(F("Clock cycles per interrupt: "));
    Serial.println(cycles_per_int);
    Serial.print(F("Interrupt frequency: "));
    Serial.print(interrupt_frequency);
    Serial.println(F(" Hz"));
    Serial.print(F("PWM frequency: "));
    Serial.print(interrupt_frequency / (maxBrightness + 1));
    Serial.println(F(" Hz"));
    
#if defined(USBCON)
    if(timerToUse == 1)
    {
        Serial.println(F("Timer1 in use."));
        Serial.println(F("Change timerToUse to 3."));
        Serial.print(F("OCR1A: "));
        Serial.println(OCR1A, DEC);
        Serial.print(F("Prescaler: "));
        Serial.println(prescaler);
        
        //Re-enable Interrupt
        bitSet(TIMSK1, OCIE1A);
    }
    else if(timerToUse == 3)
    {
        Serial.println(F("Timer3 in use."));
        Serial.print(F("OCR3A: "));
        Serial.println(OCR3A, DEC);
        Serial.print(F("Presclaler: "));
        Serial.println(prescaler);
        
        //Re-enable Interrupt
        bitSet(TIMSK3, OCIE3A);
    }
#else
    if(timerToUse==1)
    {
        Serial.println(F("Timer1 in use for highest precision."));
        Serial.println(F("Change timerToUse to 2."));
        Serial.print(F("OCR1A: "));
        Serial.println(OCR1A, DEC);
        Serial.print(F("Prescaler: "));
        Serial.println(prescaler);
        
        //Re-enable Interrupt
        bitSet(TIMSK1, OCIE1A);
    }
    else if(timerToUse == 2)
    {
        Serial.println(F("Timer2 in use."));
        Serial.print(F("OCR2A: "));
        Serial.println(OCR2A, DEC);
        Serial.print(F("Presclaler: "));
        Serial.println(prescaler);
        
        //Re-enable Interrupt
        bitSet(TIMSK2, OCIE2A);
    }
#endif
}

// OneByOne functions are usefull for testing all your outputs
void oneByOneSlow()
{
    oneByOneCore(1024 / maxBrightness);
}

void oneByOneFast()
{
    oneByOneCore(1);
}

void oneByOneCore(int delaytime)
{
    int pin,brightness;
    // Start at 0
    for(byte i = 0; i < numberOfChannels; i ++)
    {
        setBrightnessForChannel(i, 0);
    }
    
    for(int pin = 0; pin < numberOfChannels; pin++)
    {
        for(brightness = 0; brightness < maxBrightness; brightness++)
        {
            pwmValues[pin] = brightness;
            delay(delaytime);
        }
        for(brightness = maxBrightness; brightness >= 0; brightness--)
        {
            pwmValues[pin] = brightness;
            delay(delaytime);
        }
    }
}
