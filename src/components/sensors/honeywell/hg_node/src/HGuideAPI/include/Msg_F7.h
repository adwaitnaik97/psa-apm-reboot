#ifndef __HGuideAPI_Msg_F7_h__
#define __HGuideAPI_Msg_F7_h__
#pragma once

#include <cstdint>


// 0xF7 : Status Message - Operation Information
class HGUIDE_DLL Msg_F7
{
public:
	Msg_F7();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF7; // Message ID
	uint32_t hours_of_operation; // Hours of Operation
	uint32_t boot_count; // Boot Count
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F7_h__
