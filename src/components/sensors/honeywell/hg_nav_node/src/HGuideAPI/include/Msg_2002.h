#ifndef __HGuideAPI_Msg_2002_h__
#define __HGuideAPI_Msg_2002_h__
#pragma once

#include <cstdint>


// 0x2002 : GPS Configuration
class HGUIDE_DLL Msg_2002
{
public:
	Msg_2002();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 192;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2002; // Message ID
	static const uint32_t MessageLength = 48; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t GPS_Receiver_Application_Software[16];
};

#endif // __HGuideAPI_Msg_2002_h__
