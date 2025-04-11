#ifndef __HGuideAPI_Msg_B4_h__
#define __HGuideAPI_Msg_B4_h__
#pragma once

#include <cstdint>

#include <include/status_word_t.h>


// 0xB4 : Attitude Message
class HGUIDE_DLL Msg_B4
{
public:
	Msg_B4();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 26;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xB4; // Message ID
	uint16_t session; // Session ID
	double systemTov; // Time of Validity
	float attitude_i; // Attitude quaternion i component
	float attitude_j; // Attitude quaternion j component
	float attitude_k; // Attitude quaternion k component
	float attitude_s; // Attitude quaternion s component
	float temperature; // Temperature
	status_word_t status_word; // Validity Bit Field
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_B4_h__
