#ifndef __HGuideAPI_Msg_FB_h__
#define __HGuideAPI_Msg_FB_h__
#pragma once

#include <cstdint>


// 0xFB : Status Message - BIT Health
class HGUIDE_DLL Msg_FB
{
public:
	Msg_FB();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xFB; // Message ID
	uint16_t crc_fail; // CRC failure flags
	uint16_t hardware_fail; // Hardware failure flags
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_FB_h__
