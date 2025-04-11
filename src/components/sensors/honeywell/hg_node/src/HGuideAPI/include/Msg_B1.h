#ifndef __HGuideAPI_Msg_B1_h__
#define __HGuideAPI_Msg_B1_h__
#pragma once

#include <cstdint>

#include <include/status_word_t.h>


// 0xB1 : Control Message
class HGUIDE_DLL Msg_B1
{
public:
	Msg_B1();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 30;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xB1; // Message ID
	uint16_t session; // Session ID
	double systemTov; // Time of Validity
	float AngularRateX; // Angular Rate on the X axis
	float AngularRateY; // Angular Rate on the Y axis
	float AngularRateZ; // Angular Rate on the Z axis
	float LinearAccelerationX; // Specific Force along the X axis
	float LinearAccelerationY; // Specific Force along the Y axis
	float LinearAccelerationZ; // Specific Force along the Z axis
	float temperature; // Device Temperature
	status_word_t status_word; // Device status information
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_B1_h__
