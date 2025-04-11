#ifndef __HGuideAPI_Msg_B2_h__
#define __HGuideAPI_Msg_B2_h__
#pragma once

#include <cstdint>

#include <include/status_word_t.h>


// 0xB2 : Guidance Message
class HGUIDE_DLL Msg_B2
{
public:
	Msg_B2();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 42;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xB2; // Message ID
	uint16_t session; // Session ID
	double systemTov; // Time of Validity
	double DeltaAngleX; // Delta Angle on X axis
	double DeltaAngleY; // Delta Angle on Y axis
	double DeltaAngleZ; // Delta Angle on Z axis
	double DeltaVelocityX; // Delta Velocity on X axis
	double DeltaVelocityY; // Delta Velocity on Y axis
	double DeltaVelocityZ; // Delta Velocity on Z axis
	float temperature; // Temperature
	status_word_t status_word; // Device status information
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_B2_h__
