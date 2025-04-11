#ifndef __HGuideAPI_Msg_CE01_h__
#define __HGuideAPI_Msg_CE01_h__
#pragma once

#include <cstdint>


// 0xCE01 : HGuide Console Command
//
// The 0xCE01 message allows the end-user to send an ASCII string to the HGuide which
// can be interpretted by as an HGuide console commands. 
// The 0xCE01 is an input and output message:
//    * As an HGuide input, the ASCII string is sent to the HGuide console.  
//    * As an HGuide output, the 0xCE01 message contains the response for the HGuide console.
// When using 0xCE01 on ethernet interface, please select PortId = 1 / PortId = 3.
// 
class HGUIDE_DLL Msg_CE01
{
public:
	Msg_CE01();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 388;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0xCE01; // Message ID
	static const uint32_t MessageLength = 97; // Message Length
	uint32_t Checksum; // Checksum
	
	// Customer Comm Port ID:
 	// 0 = Current_Port
 	// 1 = COM1
 	// 2 = COM2
 	// 3 = COM3
 	// 4 = COM4
 	// 5 = SPI
 	// 7 = USB
 	// 9 = ETHERNET
 	// 12 = CAN
	uint32_t PortId;
	uint32_t ASCII_String_Length; // Length of ASCII String [Number of Bytes]
	
	// ASCII String [pad with zeros]
	// The string has to be ended with "\ r \ n" [0x0D 0x0A]
	uint8_t ASCII_String[360];
};

#endif // __HGuideAPI_Msg_CE01_h__
