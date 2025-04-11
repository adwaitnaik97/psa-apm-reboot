#ifndef __HGuideAPI_Msg_5109_h__
#define __HGuideAPI_Msg_5109_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5109 : GNSS Receiver Attitude/Heading Measurement Output
//
// Attitude sensed from dual antenna GNSS receiver. The attitude is refered from Main (RF1) antenna to Aux (RF2) antenna
// This message can be taken as an heading/pitch input by any HGuide INS system.
// 
class HGUIDE_DLL Msg_5109
{
public:
	Msg_5109();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 76;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5109; // Message ID
	static const uint32_t MessageLength = 19; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float Roll; // [rad] Roll
	float Pitch; // [rad] Pitch
	float True_Heading; // [rad] True Heading
	float Roll_Stdv; // [rad] Roll Stdv
	float Pitch_Stdv; // [rad] Pitch Stdv
	float True_Heading_Stdv; // [rad] True Heading Stdv
	
	// Attitude Data Source
 	// 0 = Invalid Data
 	// 1 = Dual Antenna
 	// 2 = Track over ground
	uint8_t Data_source;
	gps_mode_table_t GPSMode; // GPS Mode of operation
};

#endif // __HGuideAPI_Msg_5109_h__
