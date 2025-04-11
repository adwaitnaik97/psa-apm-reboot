#ifndef __HGuideAPI_Msg_F6_h__
#define __HGuideAPI_Msg_F6_h__
#pragma once

#include <cstdint>

#include <include/filter_config_t.h>


// 0xF6 : Status Message - Accel Configuration
class HGUIDE_DLL Msg_F6
{
public:
	Msg_F6();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF6; // Message ID
	float accel_range; // Accel Range
	filter_config_t accel_config; // Accel Filter Configuration
	uint16_t accel_sample_rate; // Accel Sample Rate
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F6_h__
