#ifndef __HGuideAPI_Msg_20FF_h__
#define __HGuideAPI_Msg_20FF_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x20FF : ACK/NAK Message
//
// Message sent in response to all input messages (unless explicitly disabled by the input message)
// 
class HGUIDE_DLL Msg_20FF
{
public:
	Msg_20FF();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 44;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x20FF; // Message ID
	static const uint32_t MessageLength = 11; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum : <br> UART : 2's complement sum such that all data, including the header, equals 0
	uint32_t Ack; // 0=NAK, 1=ACK
	uint32_t InputMessageID; // ID of the message responding to
	uint32_t NoOfValidMessagesSinceLast; // [-] number of input messages since last 0x20FF
	uint32_t NoOfValidMessagesSincePowerUp; // [-] number of input messages since power up
	double MessageTimeOfReception; // [s] INS System time of reception
};

#endif // __HGuideAPI_Msg_20FF_h__
