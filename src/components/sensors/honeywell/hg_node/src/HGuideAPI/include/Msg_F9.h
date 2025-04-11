#ifndef __HGuideAPI_Msg_F9_h__
#define __HGuideAPI_Msg_F9_h__
#pragma once

#include <cstdint>


// 0xF9 : Status Message - CPU Utilization
class HGUIDE_DLL Msg_F9
{
public:
	Msg_F9();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF9; // Message ID
	float percent_fr; // percent frame rate
	float percent_ffr; // percent filter frame rate
	float percent_cr; // percent control rate
	float percent_gr; // percent guidance rate
	float percent_100_fr; // percent 100Hz frame rate
	float percent_10_fr; // percent 10Hz frame rate
	float percent_1_fr; // percent 1Hz frame rate
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F9_h__
