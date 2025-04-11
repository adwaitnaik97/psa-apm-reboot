#ifndef __HGuideAPI_Msg_F1_h__
#define __HGuideAPI_Msg_F1_h__
#pragma once

#include <cstdint>


// 0xF1 : Status Message - Serial Number
class HGUIDE_DLL Msg_F1
{
public:
	Msg_F1();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF1; // Message ID
	uint8_t serial_number[8]; // Serial Number
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F1_h__
