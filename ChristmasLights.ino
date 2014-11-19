#include <SPI.h>

#define MAX_PACKET_SIZE 64
#define MAX_NUMBER_OF_CHANNELS 64

#define MICROSECONDS_TO_MILLISECONDS 1 / 1000.0
#define MILLISECONDS_TO_SECONDS 1 / 1000.0
#define AC__PIN 2

#define SERIAL_PRINTING // This disables most all serial printing
#ifdef SERIAL_PRINTING
    //#define DEBUG
#endif

// !!!: Variable Declarations

const byte boardID = 0x04;

//Data pin is MOSI (atmega168/328: pin 11. Mega: 51)
//Clock pin is SCK (atmega168/328: pin 13. Mega: 52)
const int ShiftPWM_latchPin = 10;
const bool ShiftPWM_invertOutputs = 0; // if invertOutputs is 1, outputs will be active low. Usefull for common anode RGB led's.

// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // include ShiftPWM.h after setting the pins!

// PWM Variables
byte maxBrightness = 254; // We use this value when interacting to save memory

// Phase Frequency Control (PFC) (Zero Cross)
unsigned long int previousZeroCrossTime = 0; // Timestamp in micros() of the latest zero crossing interrupt
unsigned long int currentZeroCrossTime = 0; // Timestamp in micros() of the previous zero crossing interrupt
unsigned long int zeroCrossTimeDifference =  0; // The calculated micros() between the last two zero crossings
float acFrequency = 60.0;
volatile byte zeroCrossTrigger= 0;

// BoardDetectValues
byte boardDetectPin = A5;
short boardDetectValue = 0;
short oldBoardDetectValue = 0;
byte numberOfBoardsConnected = 0;

// Packet Variables
const byte endOfPacketByte = 0xFF; // ASCII value 255 is our end of command byte
byte packetBuffer[MAX_PACKET_SIZE]; // buffer for serial data
byte *tempPacketBuffer = NULL;
byte tempPacketBufferIsAtPacketBufferIndex = 0;
byte theCurrentByte = 0;
byte packetBufferLength = 0;

// Channel State Variables
byte currentChannelBrightness[MAX_NUMBER_OF_CHANNELS];
byte millisecondBeforeUpdatingChannel[MAX_NUMBER_OF_CHANNELS];
byte millisecondCounterForChannel[MAX_NUMBER_OF_CHANNELS];
byte brightnessChangeForEachUpdateOfChannel[MAX_NUMBER_OF_CHANNELS]; // Bits 1-7 are data bits and bit 8 tells whether the change is positive or negative

// !!!: Method Declarations

// Packet Processing
void processPacket();
void nextByteInPacket();
void clearPacketBuffer();

// State changing methods
void turnOnChannel(byte channelNumber);
void turnOffChannel(byte channelNumber);
void turn8ChannelsOnOffStartingAtChannelNumber(byte channelStates, byte startingChannelNumber);
void setBrightnessForChannel(byte channelNumber, byte brightness);

// Board Detections
void boardDetect();
void checkNumberOfBoardsConnected();
void printNumberOfBoardsConnected();
byte boardDetectValueIsNearValueWithRange(int compareValue = 512, int range = 10);

// Zero Cross
void zeroCrossDetect();

// !!!: Setup and Main Loop

// free RAM check for debugging. SRAM for ATmega328p = 2048Kb.
int availableMemory() {
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
    pinMode(ShiftPWM_latchPin, OUTPUT);
    pinMode(boardDetectPin, INPUT);
    pinMode(AC__PIN, INPUT);
    
    // Initialize our state arrays to 0
    memset(currentChannelBrightness, 0, MAX_NUMBER_OF_CHANNELS);
    memset(millisecondBeforeUpdatingChannel, 0, MAX_NUMBER_OF_CHANNELS);
    memset(millisecondCounterForChannel, 0, MAX_NUMBER_OF_CHANNELS);
    memset(brightnessChangeForEachUpdateOfChannel, 0, MAX_NUMBER_OF_CHANNELS);
    
    // SPI Setup
    SPI.setBitOrder(LSBFIRST);
    // SPI_CLOCK_DIV2 is only a tiny bit faster in sending out the last byte.
    // SPI transfer and calculations overlap for the other bytes.
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.begin();
    
    // Serial setup
    Serial.begin(57600); // The XBee was actually running at 115200 baud, but because of inaccuracies in our timer, this value needed to be 111111
    
    // Setup the zero cross interrupt
    attachInterrupt(0, zeroCrossDetect, FALLING);
    
#ifdef SERIAL_PRINTING
    Serial.print("LD:");
    Serial.print(boardID);
    Serial.println(",Online");
#endif
}

void loop()
{
    // Detect MOSFET/TRIAC boards
    if(numberOfBoardsConnected == 0)
    {
        boardDetect();
    }
    
    // Handle AC Zero Cross
    if(zeroCrossTrigger)
    {
        handleZeroCross();
        zeroCrossTrigger= 0;
    }
    
    // Handle XBee data
	if(Serial.available())
    {
        packetBuffer[packetBufferLength] = Serial.read();
        packetBufferLength ++;
        
        // End of command byte, so now we'll proccess the command
        if(packetBuffer[packetBufferLength - 1] == endOfPacketByte)
        {
            tempPacketBuffer = packetBuffer;
            processPacket();
            clearPacketBuffer();
        }
        // Update everything so the next byte can be received
        else
        {
            // Somehow the buffer has overflowed. Try to process it and then erase it
            if(packetBufferLength > MAX_PACKET_SIZE)
            {
#ifdef SERIAL_PRINTING
                Serial.print("LD:");
                Serial.print(boardID);
                Serial.println(",Buffer overflow");
#endif
                
                processPacket();
                
                clearPacketBuffer();
            }
        }
    }
}

void handleZeroCross()
{
    // Calculate the frequency
    previousZeroCrossTime = currentZeroCrossTime;
    currentZeroCrossTime = micros();
    zeroCrossTimeDifference = currentZeroCrossTime - previousZeroCrossTime;
    acFrequency = 1.0 / (zeroCrossTimeDifference * MICROSECONDS_TO_MILLISECONDS * MILLISECONDS_TO_SECONDS);
    
    //ShiftPWM.PFCUpdate(acFrequency / 2.0); // Divide by 2 to get 60
}

// !!!: Command processing

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
    
    nextByteInPacket();
    
    // Only process the packet if it's for this board
    if(theCurrentByte == boardID)
    {
        nextByteInPacket();
        
        if(theCurrentByte == 0x01) // Command 0x01 (1 channel on)
        {
            nextByteInPacket();
            
            turnOnChannel(theCurrentByte);
        }
        else if(theCurrentByte == 0x02) // Command 0x02 (1 Channel off)
        {
            nextByteInPacket();
            
            turnOffChannel(theCurrentByte);
        }
        else if(theCurrentByte == 0x04) // Command 0x04 (1 bit state for all channels)
        {
            for(byte boardCount = 0; boardCount < numberOfBoardsConnected; boardCount ++)
            {
                nextByteInPacket();
                
                if(theCurrentByteIsLessThanTheEndOfPacketByte)
                {
                    turn8ChannelsOnOffStartingAtChannelNumber(theCurrentByte, 8 * boardCount);
                }
                // End the loop if we have somehow got to the end of packet byte
                else
                {
                    boardCount = numberOfBoardsConnected;
                }
            }
        }
        else if(theCurrentByte == 0x05) // Command 0x05 (1 channel brightness)
        {
            nextByteInPacket();
            byte channelNumber = theCurrentByte;
            
            nextByteInPacket();
            
            setBrightnessForChannel(channelNumber, theCurrentByte);
        }
        else if(theCurrentByte == 0x06) // Command 0x06 (brightness for all channels)
        {
            // Make sure all the data is here, else do nothing
            if(packetBufferLength == numberOfBoardsConnected * 8 + 3)
            {
                for(byte channelCount = 0; channelCount < numberOfBoardsConnected * 8; channelCount ++)
                {
                    nextByteInPacket();
                    
                    if(theCurrentByteIsLessThanTheEndOfPacketByte)
                    {
                        setBrightnessForChannel(channelCount, theCurrentByte);
                    }
                    // End the loop if we have somehow got to the end of packet byte
                    else
                    {
                        channelCount = numberOfBoardsConnected;
                    }
                }
            }
        }
        else if(theCurrentByte == 0x07) // Command 0x07 (Fade Channel over time in hundreths of seconds)
        {
            byte channelNumber;
            byte startBrightness;
            byte endBrightness;
            byte fadeTimeInHundrethsOfSeconds;
            byte brightnessChange;
            byte brightnessChangeIsPositive;
            unsigned int totalMillisecondsForFade;
            
            // Get the channel number
            nextByteInPacket();
            channelNumber = theCurrentByte;
            
            // Get the start and end brightness (each are only 4 bit values)
            nextByteInPacket();
            startBrightness = (theCurrentByte & 0x0F); // Get the first 4 bits
            endBrightness = (theCurrentByte & 0xF0); // Get the other 4 bits
            
            // Get the fade time
            nextByteInPacket();
            fadeTimeInHundrethsOfSeconds = theCurrentByte;
            
            // Calculate the fade information
            totalMillisecondsForFade = fadeTimeInHundrethsOfSeconds * 10;
            brightnessChange = abs(endBrightness - startBrightness);
            brightnessChangeIsPositive = (endBrightness > startBrightness ? 1 : 0);
            
            //millisecondBeforeUpdatingChannel[channelNumber] =
            
            //byte currentChannelBrightness[MAX_NUMBER_OF_CHANNELS];
            //byte millisecondBeforeUpdatingChannel[MAX_NUMBER_OF_CHANNELS];
            //byte millisecondCounterForChannel[MAX_NUMBER_OF_CHANNELS];
            //byte brightnessChangeForEachUpdateOfChannel[MAX_NUMBER_OF_CHANNELS];
        }
        else if(theCurrentByte == 0xF1) // Command 0xF1 (Request status - number of boards connected)
        {
            printNumberOfBoardsConnected();
        }
    }
}

void nextByteInPacket()
{
    theCurrentByte = tempPacketBuffer[0];
    
    tempPacketBuffer ++;
    tempPacketBufferIsAtPacketBufferIndex ++;
}

byte theCurrentByteIsLessThanTheEndOfPacketByte()
{
    if(tempPacketBufferIsAtPacketBufferIndex < packetBufferLength - 1)
    {
        return 1;
    }
    
    return 0;
}

void clearPacketBuffer()
{
	packetBufferLength = 0;
    tempPacketBuffer = NULL;
    tempPacketBufferIsAtPacketBufferIndex = 0;
	
	byte i;
	for(i = 0; i < MAX_PACKET_SIZE; i ++)
	{
		if(packetBuffer[i] != '\0')
			packetBuffer[i] = '\0';
		else
			break;
	}
}

// !!!: State changing methods

void turnOnChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, maxBrightness);
}

void turnOffChannel(byte channelNumber)
{
    setBrightnessForChannel(channelNumber, 0);
}

void turn8ChannelsOnOffStartingAtChannelNumber(byte channelStates, byte startingChannelNumber)
{
    for(byte i = 0; i < 8; i ++)
    {
        if(channelStates & (1 << i - 1)) // If the state for this channel is 1 (on)
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
    // Set the brightness
    ShiftPWM.SetOne(channelNumber, brightness);
    
#ifdef DEBUG
    Serial.print("ch:");
    Serial.print(channelNumber);
    Serial.print(" v:");
    Serial.println(brightness);
#endif
    
    // Update the channel state
    currentChannelBrightness[channelNumber] = brightness;
}

// !!!: Board Detection

void boardDetect()
{
    oldBoardDetectValue = boardDetectValue;
    boardDetectValue = analogRead(boardDetectPin);
    
    // See if there has been a change in the boardDetect Value
    if(abs(boardDetectValue - oldBoardDetectValue) > 7)
    {
        //#ifdef DEBUG
        Serial.print("bd:");
        Serial.println(boardDetectValue);
        //#endif
        
        checkNumberOfBoardsConnected();
        ShiftPWM.Start(acFrequency, maxBrightness);
        ShiftPWM.SetAmountOfRegisters(numberOfBoardsConnected);
        ShiftPWM.SetAll(0);
        
#ifdef DEBUG
        // Print Shift PWM stuff
        ShiftPWM.PrintInterruptLoad();
        
        // Print Available RAM
        Serial.print("RAM:");
        Serial.print(availableMemory());
        Serial.println(" Bytes");
#endif
    }
}

void checkNumberOfBoardsConnected()
{
    if(boardDetectValueIsNearValueWithRange(0))
    {
        numberOfBoardsConnected = 0;
        //Serial.println("NO BOARDS DETECTED! CHECK THE JUMPER!");
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 2))
    {
        numberOfBoardsConnected = 1;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 3))
    {
        numberOfBoardsConnected = 2;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 4))
    {
        numberOfBoardsConnected = 3;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 5))
    {
        numberOfBoardsConnected = 4;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 6))
    {
        numberOfBoardsConnected = 5;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 7))
    {
        numberOfBoardsConnected = 6;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 8))
    {
        numberOfBoardsConnected = 7;
    }
    else if(boardDetectValueIsNearValueWithRange(1024 / 9))
    {
        numberOfBoardsConnected = 8;
    }
    
    printNumberOfBoardsConnected();
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

void printNumberOfBoardsConnected()
{
    Serial.print("LD:");
    Serial.print(boardID);
    Serial.print(",Boards Connected:");
    Serial.println(numberOfBoardsConnected);
}

// !!!: Zero Cross (PFC) hardware interrupt

void zeroCrossDetect()
{
    zeroCrossTrigger= 1;
}
