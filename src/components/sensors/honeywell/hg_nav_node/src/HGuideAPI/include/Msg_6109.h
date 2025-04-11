#ifndef __HGuideAPI_Msg_6109_h__
#define __HGuideAPI_Msg_6109_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6109 : GNSS Receiver Attitude/Heading Measurement Output
//
// RAW attitude from GNSS receiver. The attitude is refered from Main (RF1) antenna to Aux (RF2) antenna
// This message is being outputted only when GNSS Signal is avaiable and both RF1 and RF2 are connected and separated by sufficient distance
// 
class HGUIDE_DLL Msg_6109
{
public:
	Msg_6109();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 76;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6109; // Message ID
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
	bool Data_Valid; // Data Valid
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
};

#endif // __HGuideAPI_Msg_6109_h__
