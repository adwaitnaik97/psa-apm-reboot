#ifndef __HGuideAPI_Msg_2301_h__
#define __HGuideAPI_Msg_2301_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x2301 : Autopilot Flight Control Inertial Data
//
// Inertial data output frame is defined by the  is outputted in the IMU body reference frame
// 
class HGUIDE_DLL Msg_2301
{
public:
	Msg_2301();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2301; // Message ID
	static const uint32_t MessageLength = 15; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary; // INS/GPS BIT Summary
	double angular_rate_x; // [rad/s] Angular Rate X
	double angular_rate_y; // [rad/s] Angular Rate Y
	double angular_rate_z; // [rad/s] Angular Rate Z
	double linear_acceleration_x; // [m/s/s] Linear Acceleration X
	double linear_acceleration_y; // [m/s/s] Linear Acceleration Y
	double linear_acceleration_z; // [m/s/s] Linear Acceleration Z
	uint32_t outputFrame; // 0 = IMU Body | 1 = INS Case | 2 = Vehicle Body
};

#endif // __HGuideAPI_Msg_2301_h__
