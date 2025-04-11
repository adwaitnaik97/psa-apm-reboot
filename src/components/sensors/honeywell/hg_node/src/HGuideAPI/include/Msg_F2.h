#ifndef __HGuideAPI_Msg_F2_h__
#define __HGuideAPI_Msg_F2_h__
#pragma once

#include <cstdint>


// 0xF2 : Status Message - Hardware Information
class HGUIDE_DLL Msg_F2
{
public:
	Msg_F2();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF2; // Message ID
	uint32_t part_number; // Part Number
	uint8_t hardware_version[4]; // Hardware Version
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F2_h__
