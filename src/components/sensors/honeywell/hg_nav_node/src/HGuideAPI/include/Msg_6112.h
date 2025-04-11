#ifndef __HGuideAPI_Msg_6112_h__
#define __HGuideAPI_Msg_6112_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>

// Structure defining where the information originated
struct init_sources_t
{
	uint8_t Position; // 0 = Unknown, 1 = External Input, 2 = GPS
	uint8_t Velocity; // 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Zero Velocity
	uint8_t Attitude; // 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Coarse Leveling
	uint8_t Heading; // 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Magnetometer

	void Default()
	{
		Position = 0;
		Velocity = 0;
		Attitude = 0;
		Heading = 0;
	}
};


// 0x6112 : Velocity Heading Output
//
// Pitch, roll, and heading as determined by velocity 
// Outputted only when using a single antenna
// 
class HGUIDE_DLL Msg_6112
{
public:
	Msg_6112();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 76;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6112; // Message ID
	static const uint32_t MessageLength = 19; // Message length, 19 32 bit words
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float Roll; // Spare - [rad] Roll as calculated from velocity
	float Pitch; // [rad] Pitch as calculated from velocity
	float Heading; // [rad] Yaw/True Heading as calculated from velocity
	float Roll_stdv; // Spare - [rad] Roll STDV
	float Pitch_stdv; // [rad] Pitch STDV
	float Heading_stdv; // [rad] Yaw/True Heading STDV
	init_sources_t Initialization_Sources; // Position, Velocity, Attitude and Heading Initialization_Sources
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	uint32_t Num_antennas; // Number of antennae used during Navigation Init
};

#endif // __HGuideAPI_Msg_6112_h__
