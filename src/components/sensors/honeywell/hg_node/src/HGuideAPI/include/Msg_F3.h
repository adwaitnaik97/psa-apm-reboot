#ifndef __HGuideAPI_Msg_F3_h__
#define __HGuideAPI_Msg_F3_h__
#pragma once

#include <cstdint>


// 0xF3 : Status Message - Software Information
class HGUIDE_DLL Msg_F3
{
public:
	Msg_F3();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF3; // Message ID
	uint8_t firmware_major_version; // Firmware Major Version
	uint8_t firmware_minor_version; // Firmware Minor Version
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F3_h__
