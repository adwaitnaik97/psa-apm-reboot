#ifndef __HGuideAPI_Msg_FE01_h__
#define __HGuideAPI_Msg_FE01_h__
#pragma once

#include <cstdint>

enum msg_FE01_parameter_1_t
{
	READ_REQUEST        = 0,
	READ_RESPONSE       = 1,
	READ_BEGIN          = 2,
	READ_READY          = 3,
	READ_BLOCK          = 4,
	READ_BLOCK_SUCCESS  = 5,
	READ_BLOCK_FAIL     = 6,
	READ_DONE           = 7,
	READ_SUCCESS        = 8,
	READ_FAILED         = 9,
	WRITE_REQUEST       = 10,
	WRITE_RESPONSE      = 11,
	WRITE_BEGIN         = 12,
	WRITE_READY         = 13,
	WRITE_BLOCK         = 14,
	WRITE_BLOCK_SUCCESS = 15,
	WRITE_BLOCK_FAIL    = 16,
	WRITE_DONE          = 17,
	WRITE_SUCCESS       = 18,
	WRITE_FAILED        = 19
};

// Parameter_2 is used to convey the reason why the Xfer failed
enum msg_FE01_write_fail_reason_t
{
	WRITE_FAILED_CRC_ERROR          = 1, //  CRC of received data doesn't match the one presented
	WRITE_FAILED_BLOCK_XFER_TIMEOUT = 2, //  Timeout in block transfer
	WRITE_FAILED_MISSED_BLOCK       = 4, //  Block missed in transfer - parameter 4 contains the amount of missed blocks
	WRITE_FAILED_DATA_NOT_NEW       = 8,
	WRITE_FAILED_UNEXPECTED_TYPE    = 16,
	WRITE_FAILED_DUPLICATE_BLOCK    = 32 //  Same block sent multiple times
};


// 0xFE01 : Read and Write Memory Message
//
// The 0xFE01 message allows the end-user to read and write memory from the HGuide using the HGuide Navigation Software Interface (HGNSI).
// The 0xFE01 message can be used for the following purposes:
// * Perform software updates
// * Read out software and software configuration
// * Set factory data in flash memory
// * Set customer data in flash memory
// 
class HGUIDE_DLL Msg_FE01
{
public:
	Msg_FE01();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 280;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0xFE01; // Message ID
	static const uint32_t MessageLength = 70; // Message Length
	uint32_t Checksum; // Checksum
	msg_FE01_parameter_1_t Parameter_1; // Parameter 1 an enumeration of command and response.
	uint32_t Parameter_2; // Parameter 2 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint32_t Parameter_3; // Parameter 3 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint32_t Parameter_4; // Parameter 4 depends on the value of Parameter_1. Please see the sequence diagram for more details.
	uint8_t Payload[248]; // Payload depends on the value of Parameter_1. Please see the sequence diagram for more details.
};

#endif // __HGuideAPI_Msg_FE01_h__
