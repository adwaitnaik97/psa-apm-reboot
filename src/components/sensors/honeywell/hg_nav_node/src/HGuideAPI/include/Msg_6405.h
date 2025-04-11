#ifndef __HGuideAPI_Msg_6405_h__
#define __HGuideAPI_Msg_6405_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6405 : INS Euler Attitudes
//
// This is the basic source of attitude from the INS system.
// Message contains Euler sequence (heading, pitch, roll) attitudes with associated stadard deviations
// The attitude outputted with respect to NED frame and True North. The rotation convention is from NED Frame to the Vehicle Frame.
// 
class HGUIDE_DLL Msg_6405
{
public:
	Msg_6405();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6405; // Message ID
	static const uint32_t MessageLength = 15; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float Roll; // [rad] Roll
	float Pitch; // [rad] Pitch
	float Heading; // [rad] True Heading
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float EulerAnglesSTDVRoll; // [rad] Estimated Roll Error Standard Deviation
	float EulerAnglesSTDVPitch; // [rad] Estimated Pitch Error Standard Deviation
	float EulerAnglesSTDVHeading; // [rad] Estimated True Heading Error Standard Deviation
};

#endif // __HGuideAPI_Msg_6405_h__
