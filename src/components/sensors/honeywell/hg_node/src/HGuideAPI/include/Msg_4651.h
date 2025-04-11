#ifndef __HGuideAPI_Msg_4651_h__
#define __HGuideAPI_Msg_4651_h__
#pragma once

#include <cstdint>

// enable specific real-time checks for post processing good to go
struct check_enable_t
{
	bool position; // 0 = disable, 1 = enable
	bool velocity; // 0 = disable, 1 = enable
	bool attitude; // 0 = disable, 1 = enable

	void Default()
	{
		position = 0;
		velocity = 0;
		attitude = 0;
	}
};


// 0x4651 : Real Time Post Processing Good To Go Setup
//
// The 0x4651 message is used to setup the Real Time Post Processing Good (0x6651)
// The assumption is that the user is logging the data from the INS startup.
// 
class HGUIDE_DLL Msg_4651
{
public:
	Msg_4651();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 80;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4651; // Message ID
	static const uint32_t MessageLength = 20; // Message Length
	uint32_t Checksum; // Checksum
	float WarmupTime; // [sec] Warmup time
	check_enable_t ThresholdCheckEnable; // Threshold check enable bit field
	float PosXErrorThreshold; // [m] Position X Error Threshold
	float PosYErrorThreshold; // [m] Position Y Error Threshold
	float PosZErrorThreshold; // [m] Position Z Error Threshold
	float VelXErrorThreshold; // [m/s] Velocity X Error Threshold
	float VelYErrorThreshold; // [m/s] Velocity Y Error Threshold
	float VelZErrorThreshold; // [m/s] Velocity Z Error Threshold
	float AttXErrorThreshold; // [rad] Attitude X Error Threshold
	float AttYErrorThreshold; // [rad] Attitude Y Error Threshold
	float AttZErrorThreshold; // [rad] Attitude Z Error Threshold
	uint8_t NavAxesSelector; // Navigation Axes Selector [0: Pos=NED, Vel=NED, Att=Euler | 1: Pos=ECEF, Vel=ECEF, Att=Gamma]
};

#endif // __HGuideAPI_Msg_4651_h__
