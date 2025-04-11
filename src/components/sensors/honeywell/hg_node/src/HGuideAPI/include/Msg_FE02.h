#ifndef __HGuideAPI_Msg_FE02_h__
#define __HGuideAPI_Msg_FE02_h__
#pragma once

#include <cstdint>


// 0xFE02 : Customer Data, Unique ID
//
// The 0xFE02 message allows the end-user to request the unique ID stored in the customer data section of flash memory.
// The 0xFE01 message is used to program the unique ID.
// The 0xFE02 message is used by the end-user to query the HGuide for the current unique ID.
// The 0xFE02 message is used to output the current value of the unique ID.
// Usage: Send the 0xFE02 to the HGuide with the payload set to zero and the HGuide will respond with current value of the unique ID.
// 
class HGUIDE_DLL Msg_FE02
{
public:
	Msg_FE02();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 400;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0xFE02; // Message ID
	static const uint32_t MessageLength = 100; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t Payload[348]; // Unique ID.
};

#endif // __HGuideAPI_Msg_FE02_h__
