#ifndef __HGuideAPI_Msg_9900_h__
#define __HGuideAPI_Msg_9900_h__
#pragma once

#include <cstdint>


// 0x9900 : Log Message
//
// The string contains fault's system time, severity and message
// 
class HGUIDE_DLL Msg_9900
{
public:
	Msg_9900();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 232;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x9900; // Message ID
	static const uint32_t MessageLength = 58; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t ASCII_String[160]; // ASCII String [pad with zeros]
};

#endif // __HGuideAPI_Msg_9900_h__
