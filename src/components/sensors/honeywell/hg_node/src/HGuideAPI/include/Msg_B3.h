#ifndef __HGuideAPI_Msg_B3_h__
#define __HGuideAPI_Msg_B3_h__
#pragma once

#include <cstdint>

#include <include/status_word_t.h>


// 0xB3 : Magnetometer Message
class HGUIDE_DLL Msg_B3
{
public:
	Msg_B3();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 24;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xB3; // Message ID
	uint16_t session; // Session ID
	double systemTov; // Time of Validity
	float MagneticFieldX; // Magnetic field along X axis
	float MagneticFieldY; // Magnetic field along Y axis
	float MagneticFieldZ; // Magnetic field along Z axis
	float temperature; // Temperature
	status_word_t status_word; // Validity Bit Field
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_B3_h__
