#ifndef __HGuideAPI_Msg_F4_h__
#define __HGuideAPI_Msg_F4_h__
#pragma once

#include <cstdint>


// 0xF4 : Status Message - Port Information
class HGUIDE_DLL Msg_F4
{
public:
	Msg_F4();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF4; // Message ID
	uint32_t baud_rate; // Baud Rate
	uint32_t bytes_transmitted; // Bytes transmitted since boot
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F4_h__
