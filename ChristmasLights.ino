#include "pins_arduino_compile_time.h"
#include <SPI.h>
#include <Arduino.h>

#pragma mark - Variable Declarations

// Debugging defines
#define SERIAL_PRINTING // This disables most all serial printing
//#define DEBUG // Outputs extra logs to Serial
//#define TESTING // Uncomment to enable all channel testing

// Board specific variables! Change these per board!
const byte boardID = 0x05;
byte numberOfShiftRegisters = 2;

/////////////////////////////////////////////////////////////////////////////////////////////
// Don't change any variables below here unless you really, really know what you are doing //
/////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_PACKET_SIZE 32

#define MICROSECONDS_TO_MILLISECONDS 1 / 1000.0
#define MILLISECONDS_TO_SECONDS 1 / 1000.0

// workaround for a bug in WString.h
#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))

// The macro below uses 3 instructions per pin to generate the byte to transfer with SPI for the shift registers. Retreive duty cycle setting from memory (ldd, 2 clockcycles). Compare with the counter (cp, 1 clockcycle) --> result is stored in carry. Use the rotate over carry right to shift the compare result into the byte. (1 clockcycle).
// TL;DR super fast way of doing the following
// if(counter < pwmVal)
//      turnOn;
// else
//      turnOff;
#define add_one_pin_to_byte(sendbyte, counter, tempPWMValues) \
{ \
byte pwmval=*tempPWMValues; \
asm volatile ("cp %0, %1" : /* No outputs */ : "r" (counter), "r" (pwmval): ); \
asm volatile ("ror %0" : "+r" (sendbyte) : "r" (sendbyte) : ); 	\
}

// Phase Frequency Control (PFC) (Zero Cross)
float pwmFrequency = 120;
unsigned long int previousZeroCrossTime = 0; // Timestamp in micros() of the latest zero crossing interrupt
unsigned long int currentZeroCrossTime = 0; // Timestamp in micros() of the previous zero crossing interrupt
unsigned long nominalZeroCrossTimeDifference = (unsigned long)(1000000 / pwmFrequency); // This is the default expected time between zero crosses
int acceptableZeroCrossDeviationInMicroseconds = 400; // How much the zero crosses can change by before taking action
unsigned long int zeroCrossTimeDifference =  nominalZeroCrossTimeDifference; // The calculated micros() between the last two zero crossings
unsigned long int averageZeroCrossTimeDifference =  nominalZeroCrossTimeDifference; // The calculated micros() between the last two zero crossings
byte zeroCrossPin = 2;

// Shift Register Detect Variables
byte shiftRegisterDetectPin = A5;
short shiftRegisterDetectValue = 0;
short oldShiftRegisterDetectValue = 0;

// Serial Packet Variables
const byte endOfPacketByte = 0xFF; // ASCII value 255 is our end of command byte
byte packetBuffer[MAX_PACKET_SIZE]; // buffer for serial data
byte packetBufferLength = 0;
byte currentByteFromPacket = 0;
byte currentByteIndex = 0;

// Shift PWM Pins
const int latchPin = 10;
volatile uint8_t *latchPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[latchPin]];
const uint8_t latchBit =  digital_pin_to_bit_PGM_ct[latchPin];
volatile uint8_t *clockPort = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[SCK]];
const uint8_t clockBit = digital_pin_to_bit_PGM_ct[SCK];
volatile uint8_t *dataPort  = port_to_output_PGM_ct[digital_pin_to_port_PGM_ct[MOSI]];
const uint8_t dataBit = digital_pin_to_bit_PGM_ct[MOSI];

// Shift Register
byte maxBrightness = 127;
byte numberOfChannels = 0;
byte *pwmValues = 0;
volatile byte shiftRegisterCurrentBrightnessIndex = maxBrightness;

// Dimming variables
float *brightnessChangePerDimmingCycle = 0;
float *temporaryPWMValues = 0;
unsigned short *dimmingUpdatesCount = 0;
byte *finalPWMValues = 0;
volatile byte updateDimming = 0;
#ifdef TESTING
    byte *dimmingDirection; // For testing only
#endif

#pragma mark - Method Declarations

// Packet Processing
void processPacket();
byte readNextByteInPacket();
void clearPacketBuffer();

// State changing methods
void turnOnChannel(byte channelNumber);
void turnOffChannel(byte channelNumber);
void setBrightnessForChannel(byte channelNumber, byte brightness);
void fadeChannelNumberToBrightnessWithMillisecondsDuration(byte channelNumber, byte brightness, unsigned long milliseconds);
bool isChannelNumberValid(byte channelNumber);

// Shift Register Detection Methods
void detectShiftRegisters();
byte shiftRegisterDetectValueIsNearValueWithRange(int compareValue = 512, int range = 10);

// Zero Cross
void zeroCrossDetect();

// ShiftRegister
void handleShiftRegisterTimerInterrupt();
void initShiftRegisterInterruptTimer();
void printInterruptLoad();

// Dimming
void dimmingUpdate();

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
    pinMode(shiftRegisterDetectPin, INPUT);
    pinMode(zeroCrossPin, INPUT);
    
    // Initialize pin states
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
    
    // Initialize shift Register Interrupt Timer (timer 1)
    if(isInterruptLoadAcceptable())
    {
        initShiftRegisterInterruptTimer();
    }
    else
    {
        Serial.println(F("Interrupts are disabled because load is too high."));
        cli(); //Disable interrupts
    }
    
    // Setup the zero cross interrupt which uses zeroCrossPin (zeroCrossPin can't be changed though)
    attachInterrupt(0, zeroCrossDetect, FALLING);
    
    initializeWithewShiftRegisterCount();
    
#ifdef SERIAL_PRINTING
    Serial.print("LD:");
    Serial.print(boardID);
    Serial.println(",Online");
#endif
}

void loop()
{
    // Handle dimming
    if(updateDimming)
    {
        dimmingUpdate();
        updateDimming = 0;
    }
    
    // Handle XBee data
	if(Serial.available())
    {
        // Read in a byte and store it
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
    
    // Detect MOSFET/TRIAC boards
    if(numberOfShiftRegisters == 0)
    {
        detectShiftRegisters();
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
    
    // Read in the boardID byte
    byte boardIDByte = readNextByteInPacket();
    
    // Only process the packet if it's for this board
    if(boardIDByte == boardID)
    {
        // Read in the commandID byte
        byte commandID = readNextByteInPacket();
        
        if(commandID == 0x01) // Command 0x00 (1 channel on)
        {
            turnOnChannel(readNextByteInPacket());
        }
        else if(commandID == 0x02) // Command 0x01 (1 Channel off)
        {
            turnOffChannel(readNextByteInPacket());
        }
        else if(commandID == 0x04) // Command 0x04 (1 bit state for all channels)
        {
            byte startingIndex = 0;
            // Loop through only the data bytes (skip the end of packet byte)
            while(currentByteIndex < packetBufferLength - 1)
            {
                turn8ChannelsOnOffStartingAtChannelNumber(startingIndex * 8, readNextByteInPacket());
                startingIndex ++;
            }
        }
        else if(commandID == 0x05) // Command 0x05 (1 Channel on for time in hundreths)
        {
            byte channelNumber = readNextByteInPacket();
            byte fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
            channelBrightnessWithMillisecondsDuration(channelNumber, maxBrightness, fadeTimeInHundrethsOfSeconds * 10);
            
        }
        else if(commandID == 0x06) // Command 0x06 (1 Channel on for time in tenths)
        {
            byte channelNumber = readNextByteInPacket();
            byte fadeTimeInTenthsOfSeconds = readNextByteInPacket();
            channelBrightnessWithMillisecondsDuration(channelNumber, maxBrightness, fadeTimeInTenthsOfSeconds * 100);
            
        }
        else if(commandID == 0x10) // Command 0x10 (1 brightness)
        {
            byte channelNumber = readNextByteInPacket();
            byte brightness = readNextByteInPacket();
            setBrightnessForChannel(channelNumber, brightness);
        }
        else if(commandID == 0x11) // Command 0x11 (All brightness)
        {
            byte brightness = readNextByteInPacket();
            
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                setBrightnessForChannel(i, brightness);
            }
        }
        else if(commandID == 0x12) // Command 0x12 (1 Channel brightness for time in hundreths)
        {
            byte channelNumber = readNextByteInPacket();
            byte brightness = readNextByteInPacket();
            byte fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
            channelBrightnessWithMillisecondsDuration(channelNumber, brightness, fadeTimeInHundrethsOfSeconds * 10);
            
        }
        else if(commandID == 0x13) // Command 0x13 (1 Channel brightness for time in tenths)
        {
            byte channelNumber = readNextByteInPacket();
            byte brightness = readNextByteInPacket();
            byte fadeTimeInTenthsOfSeconds = readNextByteInPacket();
            channelBrightnessWithMillisecondsDuration(channelNumber, brightness, fadeTimeInTenthsOfSeconds * 100);
            
        }
        else if(commandID == 0x15) // Command 0x15 (All on)
        {
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                turnOnChannel(i);
            }
        }
        else if(commandID == 0x16) // Command 0x16 (All off)
        {
            for(byte i = 0; i < numberOfChannels; i ++)
            {
                turnOffChannel(i);
            }
        }
        // Fade channel/all
        else if(commandID == 0x20 || commandID == 0x21 || commandID == 0x22 || commandID == 0x23 || commandID == 0x24 || commandID == 0x25 || commandID == 0x30 || commandID == 0x31 || commandID == 0x32 || commandID == 0x33 || commandID == 0x34 || commandID == 0x35)
        {
            byte channelNumber;
            byte startBrightness;
            byte endBrightness;
            byte fadeTimeInHundrethsOfSeconds;
            byte fadeTimeInTenthsOfSeconds;
            
            // Get the channel number for single channel commands
            if(commandID == 0x20 || commandID == 0x21 || commandID == 0x22 || commandID == 0x23 || commandID == 0x24 || commandID == 0x25)
            {
                channelNumber = readNextByteInPacket();
            }
            
            // Fade up
            if(commandID == 0x20 || commandID == 0x21 || commandID == 0x30 || commandID == 0x31)
            {
                // Get the end brightness
                startBrightness = 0;
                endBrightness = maxBrightness;
            }
            // Fade down
            if(commandID == 0x22 || commandID == 0x23 || commandID == 0x32 || commandID == 0x33)
            {
                // Get the end brightness
                startBrightness = maxBrightness;
                endBrightness = 0;
            }
            // Fade from x to y
            if(commandID == 0x24 || commandID == 0x25 || commandID == 0x34 || commandID == 0x35)
            {
                // Get the startBrightness
                startBrightness = readNextByteInPacket();
                if(startBrightness > maxBrightness)
                {
                    startBrightness = maxBrightness;
                }
                else if(startBrightness < 0)
                {
                    startBrightness = 0;
                }
                
                // Get the endBrightness
                endBrightness = readNextByteInPacket();
                if(endBrightness > maxBrightness)
                {
                    endBrightness = maxBrightness;
                }
                else if(endBrightness < 0)
                {
                    endBrightness = 0;
                }
            }
            
            // Fade channel in hundreths
            if(commandID == 0x20 || commandID == 0x22 || commandID == 0x24)
            {
                fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(channelNumber, startBrightness, endBrightness, fadeTimeInHundrethsOfSeconds * 10);
            }
            // Fade channel in tenths
            else if(commandID == 0x21 || commandID == 0x23 || commandID == 0x25)
            {
                fadeTimeInTenthsOfSeconds = readNextByteInPacket();
                // Set the fade
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(channelNumber, startBrightness, endBrightness, fadeTimeInTenthsOfSeconds * 100);
            }
            // Fade all hundredths
            else if(commandID == 0x30 || commandID == 0x32 || commandID == 0x34)
            {
                fadeTimeInHundrethsOfSeconds = readNextByteInPacket();
                // Set the fade
                for(byte i = 0; i < numberOfChannels; i ++)
                {
                    fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, startBrightness, endBrightness, fadeTimeInHundrethsOfSeconds * 10);
                }
            }
            // Fade all tenths
            else if(commandID == 0x31 || commandID == 0x33 || commandID == 0x35)
            {
                fadeTimeInTenthsOfSeconds = readNextByteInPacket();
                // Set the fade
                for(byte i = 0; i < numberOfChannels; i ++)
                {
                    fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, startBrightness, endBrightness, fadeTimeInTenthsOfSeconds * 100);
                }
            }
        }
        else if(commandID == 0xF1) // Command 0xF1 (Request status - number of boards connected)
        {
            Serial.println(numberOfShiftRegisters);
        }
    }
}

byte readNextByteInPacket()
{
    // If we aren't past the end of our buffer, get the next buffered byte
    if(currentByteIndex < packetBufferLength)
    {
        currentByteFromPacket = packetBuffer[currentByteIndex];
    }
    else
    {
        currentByteFromPacket = 0;
    }
    
    currentByteIndex ++;
    
    return currentByteFromPacket;
}

void clearPacketBuffer()
{
	packetBufferLength = 0;
    currentByteIndex = 0;
	
    // Set our packetBuffer to all 0's (nils)
	memset(packetBuffer, 0, MAX_PACKET_SIZE * sizeof(byte));
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

void turn8ChannelsOnOffStartingAtChannelNumber(byte startingChannelNumber, byte channelStates)
{
    for(byte i = 0; i < 8; i ++)
    {
        if(channelStates & (1 << i)) // If the state for this channel is 1 (off)
        {
            setBrightnessForChannel((startingChannelNumber + i), maxBrightness);
        }
        else
        {
            setBrightnessForChannel((startingChannelNumber + i), 0);
        }
    }
}

void setBrightnessForChannel(byte channelNumber, byte brightness)
{
    // Set the brightness if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        pwmValues[channelNumber] = brightness;
        // Cancel a fade if it was fading
        dimmingUpdatesCount[channelNumber] = 0;
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" v:");
    Serial.println(brightness);
#endif
}

void channelBrightnessWithMillisecondsDuration(byte channelNumber, byte brightness, unsigned long milliseconds)
{
    // Set the brightnessChangePerDimmingCycle if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        pwmValues[channelNumber] = brightness;
        
        dimmingUpdatesCount[channelNumber] = milliseconds / (1000.0 / pwmFrequency) + 1;
        brightnessChangePerDimmingCycle[channelNumber] = 0;
        temporaryPWMValues[channelNumber] = pwmValues[channelNumber];
        finalPWMValues[channelNumber] = 0;
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" p:");
    Serial.println(brightnessChangePerDimmingCycle[channelNumber]);
#endif
}

void fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(byte channelNumber, byte startBrightness, byte endBrightness, unsigned long milliseconds)
{
    // Set the brightnessChangePerDimmingCycle if the channel is valid
    if(isChannelNumberValid(channelNumber))
    {
        pwmValues[channelNumber] = startBrightness;
        
        dimmingUpdatesCount[channelNumber] = milliseconds / (1000.0 / pwmFrequency) + 1;
        brightnessChangePerDimmingCycle[channelNumber] = (float)(endBrightness - startBrightness) / dimmingUpdatesCount[channelNumber];
        temporaryPWMValues[channelNumber] = pwmValues[channelNumber];
        finalPWMValues[channelNumber] = endBrightness;
    }
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" p:");
    Serial.println(brightnessChangePerDimmingCycle[channelNumber]);
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
        Serial.print(F("Error: Trying to change channel "));
        Serial.println(channelNumber);
        Serial.println(F(" that isn't initialized"));
        return 0;
    }
}

#pragma mark - Shift Register/Board Detection

void detectShiftRegisters()
{
    // Read in the values for the board detect
    oldShiftRegisterDetectValue = shiftRegisterDetectValue;
    shiftRegisterDetectValue = analogRead(shiftRegisterDetectPin);
    
    // See if there has been a change in the boardDetect Value
    if(abs(shiftRegisterDetectValue - oldShiftRegisterDetectValue) > 7)
    {
        // Set the number of shift registers appropriatly
        if(shiftRegisterDetectValueIsNearValueWithRange(0))
        {
            numberOfShiftRegisters = 0;
            Serial.println("NO BOARDS DETECTED! CHECK THE JUMPER!");
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 2))
        {
            numberOfShiftRegisters = 1;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 3))
        {
            numberOfShiftRegisters = 2;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 4))
        {
            numberOfShiftRegisters = 3;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 5))
        {
            numberOfShiftRegisters = 4;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 6))
        {
            numberOfShiftRegisters = 5;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 7))
        {
            numberOfShiftRegisters = 6;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 8))
        {
            numberOfShiftRegisters = 7;
        }
        else if(shiftRegisterDetectValueIsNearValueWithRange(1024 / 9))
        {
            numberOfShiftRegisters = 8;
        }
        
#ifdef DEBUG
        Serial.print("bd:");
        Serial.println(shiftRegisterDetectValue);
        Serial.print("boards:");
        Serial.println(numberOfShiftRegisters);
#endif
        
        initializeWithewShiftRegisterCount();
        
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

byte shiftRegisterDetectValueIsNearValueWithRange(int compareValue, int range)
{
    if(shiftRegisterDetectValue >= compareValue - range && shiftRegisterDetectValue <= compareValue + range)
    {
        return 1;
    }
    else if(compareValue < range && shiftRegisterDetectValue < range)
    {
        return 1;
    }
    else if(compareValue > 1024 - range && shiftRegisterDetectValue > 1024 - range)
    {
        return 1;
    }
    
    return 0;
}

void initializeWithewShiftRegisterCount()
{
    // Disable interrupt
    cli();
    // Update the number of channels
    numberOfChannels = numberOfShiftRegisters * 8;
    
    // Check if new amount will not result in deadlock
    if(isInterruptLoadAcceptable())
    {
        // Malloc pwmValues array
        pwmValues = (byte *)malloc(numberOfChannels * sizeof(byte));
        // Initialize pwmValues array to 0
        memset(pwmValues, 0, numberOfChannels * sizeof(byte));
        
        // Malloc finalPWMValues array
        finalPWMValues = (byte *)malloc(numberOfChannels * sizeof(byte));
        // Initialize pwmValues array to 0
        memset(finalPWMValues, 0, numberOfChannels * sizeof(byte));
        
        // Malloc temporaryPWMValues array
        temporaryPWMValues = (float *)malloc(numberOfChannels * sizeof(float));
        // Initialize temporaryPWMValues array to 0
        memset(temporaryPWMValues, 0, numberOfChannels * sizeof(float));
        
        // Malloc brightnessChangePerDimmingCycle array
        brightnessChangePerDimmingCycle = (float *)malloc(numberOfChannels * sizeof(float));
        // Initialize brightnessChangePerDimmingCycle array to 0
        memset(brightnessChangePerDimmingCycle, 0, numberOfChannels * sizeof(float));
        
        // Malloc dimmingUpdatesCount array
        dimmingUpdatesCount = (unsigned short *)malloc(numberOfChannels * sizeof(unsigned short));
        // Initialize dimmingUpdatesCount array to 0
        memset(dimmingUpdatesCount, 0, numberOfChannels * sizeof(unsigned short));
        
#ifdef TESTING
        // Malloc dimmingDirection array
        dimmingDirection = (byte *)malloc(numberOfChannels * sizeof(byte));
        // Initialize dimmingDirection array to 0
        memset(dimmingDirection, 0, numberOfChannels * sizeof(byte));
#endif
        
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
}

#pragma mark - Zero Cross

// Hardware zero cross interrupt
void zeroCrossDetect()
{
    handleZeroCross();
}

// Handle the zero cross
void handleZeroCross()
{
    // Calculate the frequency
    previousZeroCrossTime = currentZeroCrossTime;
    currentZeroCrossTime = micros();
    zeroCrossTimeDifference = currentZeroCrossTime - previousZeroCrossTime;
    averageZeroCrossTimeDifference -= averageZeroCrossTimeDifference / pwmFrequency;
    averageZeroCrossTimeDifference += zeroCrossTimeDifference / pwmFrequency;
    
    // Only update the pwmFrequency if it has changed by ~1 hz
    if(averageZeroCrossTimeDifference > nominalZeroCrossTimeDifference + acceptableZeroCrossDeviationInMicroseconds || averageZeroCrossTimeDifference < nominalZeroCrossTimeDifference - acceptableZeroCrossDeviationInMicroseconds)
    {
        // Update he frequency
        pwmFrequency = 1.0 / (averageZeroCrossTimeDifference * MICROSECONDS_TO_MILLISECONDS * MILLISECONDS_TO_SECONDS);
        // Update the shift register interrupt timer
        OCR1A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
        // Also update what we expect the zero cross time difference
        nominalZeroCrossTimeDifference = averageZeroCrossTimeDifference;
    }
    
    // Keep the shift register timer in phase with the zero cross
    TCNT1 = 0;
    
    // Handle AC dimming (fading over time)
    updateDimming = 1;
}

#pragma mark - Shift Register

//Install the Interrupt Service Routine (ISR) for Timer1 compare and match A.
ISR(TIMER1_COMPA_vect)
{
    handleShiftRegisterTimerInterrupt();
}

void initShiftRegisterInterruptTimer()
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
    
    // The timer will generate an interrupt when the value we load in OCR1A matches the timer value. One period of the timer, from 0 to OCR1A will therefore be (OCR1A + 1) / timer clock frequency. We want the frequency of the timer to be (pwmFrequency) * (maxBrightness). So the value we want for OCR1A to be: timer clock frequency / (pwmFrequency * maxBrightness) - 1
    OCR1A = round((float)F_CPU / (pwmFrequency * ((float)maxBrightness + 1))) - 1;
    
    // Enable the timer interrupt, see datasheet  15.11.8)
    bitSet(TIMSK1, OCIE1A);
}

void handleShiftRegisterTimerInterrupt()
{
    // Define a pointer that will be used to access the values for each output. Let it point one past the last value, because it is decreased before it is used.
    byte *tempPWMValues = &pwmValues[numberOfChannels];
    
    // Write shift register latch clock low
    bitClear(*latchPort, latchBit);
    
    // Do a whole shift register at once. This unrolls the loop for extra speed
    for(unsigned char i = numberOfShiftRegisters; i > 0; --i)
    {
        unsigned char sendbyte;  // no need to initialize, all bits are replaced
        
        // Build the byte. One bit for each channel in this shift register
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        add_one_pin_to_byte(sendbyte, shiftRegisterCurrentBrightnessIndex, --tempPWMValues);
        
        // Send the byte to the SPI
        SPDR = sendbyte;
        
        // wait for last send to finish and retreive answer. Retreive must be done, otherwise the SPI will not work.
        while (!(SPSR & _BV(SPIF)));
    }
    
    // Write shift register latch clock high
    bitSet(*latchPort, latchBit);
    
    // Decrease the brightness index. This is the key to getting AC dimming since triacs stay on until the next zero cross. So we need to turn on after a delay rather than turn on immediately and turn off after a delay. AKA, this needs to count down not up. Trust me!
    if(shiftRegisterCurrentBrightnessIndex > 0)
    {
        shiftRegisterCurrentBrightnessIndex --;
    }
    else
    {
        // Handle DC dimming (fading over time)
        updateDimming = 1;
    }
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

void printInterruptLoad()
{
    //This function prints information on the interrupt settings for ShiftRegisterPWM. It runs a delay loop 2 times: once with interrupts enabled, once disabled. From the difference in duration, it can calculate the load of the interrupt on the program.
    unsigned long start1,end1,time1,start2,end2,time2,k;
    double load, cycles_per_int, interrupt_frequency;
    
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
    
    //run with interrupt enabled
    start1 = micros();
    for(k = 0; k < 100000; k++)
    {
        delayMicroseconds(1);
    }
    end1 = micros();
    time1 = end1 - start1;
    
    //Disable Interrupt
    bitClear(TIMSK1, OCIE1A);
    
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
    interrupt_frequency = F_CPU / (OCR1A + 1);
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
    Serial.println(F("Timer1 in use for highest precision."));
    Serial.print(F("OCR1A: "));
    Serial.println(OCR1A, DEC);
    
    //Re-enable Interrupt
    bitSet(TIMSK1, OCIE1A);
}

#pragma mark - Dimming

void dimmingUpdate()
{
    // Reset the brightness index to be in phase with the zero cross
    shiftRegisterCurrentBrightnessIndex = maxBrightness;
    
#ifdef TESTING
    dimmingTest();
#endif
    
    // Handle the dimming over time for each channel
    for(byte i = 0; i < numberOfChannels; i ++)
    {
        // Update a channel if it is still dimming
        if(dimmingUpdatesCount[i] > 0)
        {
            temporaryPWMValues[i] += brightnessChangePerDimmingCycle[i];
            pwmValues[i] = temporaryPWMValues[i];
            dimmingUpdatesCount[i] --;
            if(dimmingUpdatesCount[i] == 0)
            {
                pwmValues[i] = finalPWMValues[i];
            }
        }
    }
}

#ifdef TESTING

void dimmingTest()
{
    for(byte i = 0; i < numberOfChannels; i ++)
    {
        if(dimmingUpdatesCount[i] == 0)
        {
            if(dimmingDirection[i] == 1)
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, maxBrightness, 0, 500 + i * 100);
                dimmingDirection[i] = 0;
            }
            else
            {
                fadeChannelNumberFromBrightnessToBrightnessWithMillisecondsDuration(i, 0, maxBrightness, 500 + i * 100);
                dimmingDirection[i] = 1;
            }
        }
    }
}

#endif
