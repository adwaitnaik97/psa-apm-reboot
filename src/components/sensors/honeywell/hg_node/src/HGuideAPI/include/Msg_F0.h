#ifndef __HGuideAPI_Msg_F0_h__
#define __HGuideAPI_Msg_F0_h__
#pragma once

#include <cstdint>


// 0xF0 : Status Message - Device Type
//
// Message contains device type information
// 
class HGUIDE_DLL Msg_F0
{
public:
	Msg_F0();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF0; // Message ID
	uint8_t device_type[4]; // Device type
	uint8_t device_config[4]; // Device configuration
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F0_h__
