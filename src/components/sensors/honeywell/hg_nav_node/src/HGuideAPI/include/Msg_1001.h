#ifndef __HGuideAPI_Msg_1001_h__
#define __HGuideAPI_Msg_1001_h__
#pragma once

#include <cstdint>

#include <include/hgnsi_message_word1_t.h>
#include <include/hgnsi_message_word2_t.h>
#include <include/hgnsi_message_word3_t.h>
#include <include/hgnsi_message_word4_t.h>


// 0x1001 : Enable/Disable Output Messages
//
// The 0x1001 message can be used to enable and disable output messages.  
// The message consists of 32-bit bitfields which enable a message if the bit is 
// set to one (1 / True) and disable the message if the bit is set to zero (0 / False).
// 
class HGUIDE_DLL Msg_1001
{
public:
	Msg_1001();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1001; // Message ID
	static const uint32_t MessageLength = 16; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	hgnsi_message_word1_t MessageWord1; // Word 1 with SAVE DATA TO MEMORY and message enable bits
	hgnsi_message_word2_t MessageWord2; // Word 2 with message enable bits
	hgnsi_message_word3_t MessageWord3; // Word 3 with message enable bits
	hgnsi_message_word4_t MessageWord4; // Word 4 with message enable bits
};

#endif // __HGuideAPI_Msg_1001_h__
