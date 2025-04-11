#ifndef __HGuideAPI_Msg_F8_h__
#define __HGuideAPI_Msg_F8_h__
#pragma once

#include <cstdint>


// 0xF8 : Status Message - Power Information
class HGUIDE_DLL Msg_F8
{
public:
	Msg_F8();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xF8; // Message ID
	float device_current; // Device Input Current
	float device_voltage; // Device Input Voltage
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_F8_h__
