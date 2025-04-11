#ifndef __HGuideAPI_Msg_6428_h__
#define __HGuideAPI_Msg_6428_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6428 : GNSS Normalized Measurement Residuals
//
// Residuals of GNSS measurements of position, velocity and attitude
// 
class HGUIDE_DLL Msg_6428
{
public:
	Msg_6428();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 180;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6428; // Message ID
	static const uint32_t MessageLength = 45; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // [-] GPS Week no.
	float gps_pvt_pos_x; // Word 11 :
	float gps_pvt_pos_y; // Word 12 :
	float gps_pvt_pos_z; // Word 13 :
	float gps_pvt_vel_x; // Word 14 :
	float gps_pvt_vel_y; // Word 15 :
	float gps_pvt_vel_z; // Word 16 :
	float gnss_att_x; // Word 17 :
	float gnss_att_y; // Word 18 :
	float gnss_att_z; // Word 19 :
};

#endif // __HGuideAPI_Msg_6428_h__
