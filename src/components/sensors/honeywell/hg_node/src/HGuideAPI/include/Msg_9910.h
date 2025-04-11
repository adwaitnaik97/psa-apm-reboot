#ifndef __HGuideAPI_Msg_9910_h__
#define __HGuideAPI_Msg_9910_h__
#pragma once

#include <cstdint>


// 0x9910 : Processor Loading
//
// Contains the processor loading for the INS SW threads.
// 
class HGUIDE_DLL Msg_9910
{
public:
	Msg_9910();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 44;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x9910; // Message ID
	static const uint32_t MessageLength = 11; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t own_usage_rate_0; // Rate 0 thread's own usage [%]
	uint8_t own_usage_rate_1; // Rate 1 thread's own usage [%]
	uint8_t own_usage_rate_2; // Rate 2 thread's own usage [%]
	uint8_t own_usage_rate_3; // Rate 3 thread's own usage [%]
	uint8_t own_usage_rate_4; // Rate 4 thread's own usage [%]
	uint8_t own_usage_rate_5; // Rate 5 thread's own usage [%]
	uint8_t own_usage_rate_6; // Rate 6 thread's own usage [%]
	uint8_t own_usage_rate_7; // Rate 7 thread's own usage [%]
	uint8_t total_usage_rate_0; // Rate 0 thread's total usage [%]
	uint8_t total_usage_rate_1; // Rate 1 thread's total usage [%]
	uint8_t total_usage_rate_2; // Rate 2 thread's total usage [%]
	uint8_t total_usage_rate_3; // Rate 3 thread's total usage [%]
	uint8_t total_usage_rate_4; // Rate 4 thread's total usage [%]
	uint8_t total_usage_rate_5; // Rate 5 thread's total usage [%]
	uint8_t total_usage_rate_6; // Rate 6 thread's total usage [%]
	uint8_t total_usage_rate_7; // Rate 7 thread's total usage [%]
	uint8_t max_usage_rate_0; // Rate 0 thread's max usage [%]
	uint8_t max_usage_rate_1; // Rate 1 thread's max usage [%]
	uint8_t max_usage_rate_2; // Rate 2 thread's max usage [%]
	uint8_t max_usage_rate_3; // Rate 3 thread's max usage [%]
	uint8_t max_usage_rate_4; // Rate 4 thread's max usage [%]
	uint8_t max_usage_rate_5; // Rate 5 thread's max usage [%]
	uint8_t max_usage_rate_6; // Rate 6 thread's max usage [%]
	uint8_t max_usage_rate_7; // Rate 7 thread's max usage [%]
	uint8_t total_usage; // Total usage accross all threads [%]
	uint8_t max_usage; // Maximum usage accross all threads [%]
};

#endif // __HGuideAPI_Msg_9910_h__
