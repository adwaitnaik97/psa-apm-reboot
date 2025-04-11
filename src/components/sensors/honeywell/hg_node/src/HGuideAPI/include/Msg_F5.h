#ifndef __HGuideAPI_Msg_F5_h__
#define __HGuideAPI_Msg_F5_h__
#pragma once

#include <cstdint>

#include <include/filter_config_t.h>


// 0xF5 : Status Message - Gyro Configuration
class HGUIDE_DLL Msg_F5
{
public:
	Msg_F5();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF5; // Message ID
	float gyro_range; // Gyro Range
	filter_config_t gyro_config; // Gyro Filter Configuration
	uint16_t gyro_sample_rate; // Gyro Sample Rate
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F5_h__
