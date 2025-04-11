#ifndef __HGuideAPI_Msg_6504_h__
#define __HGuideAPI_Msg_6504_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6504 : INS NED Velocity
//
// This is the basic source of velocity from the INS system.
// Contains North East Down Velocity with standard deviations
// The velocity is referenced from Vehicle Frame
// 
class HGUIDE_DLL Msg_6504
{
public:
	Msg_6504();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6504; // Message ID
	static const uint32_t MessageLength = 15; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // System Time of Validity [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float NorthVelocity; // North Velocity [m/s]
	float EastVelocity; // East Velocity [m/s]
	float DownVelocity; // Down Velocity [m/s]
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Mode Summary
	float NorthVelocitySTDV; // Estimated North Velocity Error Standard Deviation (1-sigma) [m/s]
	float EastVelocitySTDV; // Estimated East Velocity Error Standard Deviation (1-sigma) [m/s]
	float DownVelocitySTDV; // Estimated Down Velocity Error Standard Deviation (1-sigma) [m/s]
};

#endif // __HGuideAPI_Msg_6504_h__
