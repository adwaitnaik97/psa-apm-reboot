#ifndef __HGuideAPI_Msg_6424_h__
#define __HGuideAPI_Msg_6424_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6424 : GNSS Measurement Error Estimates and Uncertainty
//
// Standard deviations of various GNSS measurements (PVT lever arms, Antenna Boresight, GNSS position)
// 
class HGUIDE_DLL Msg_6424
{
public:
	Msg_6424();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 168;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6424; // Message ID
	static const uint32_t MessageLength = 42; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // [-] GPS Week no.
	float gps_pvt_lever_arm_x; // Word 22 :
	float gps_pvt_lever_arm_y; // Word 23 :
	float gps_pvt_lever_arm_z; // Word 24 :
	float gnss_att_brsght_state_x; // Word 25 :
	float gnss_att_brsght_state_y; // Word 26 :
	float gnss_att_brsght_state_z; // Word 27 :
	float gnss_att_brsght_x_stdv; // Word 28 :
	float gnss_att_brsght_y_stdv; // Word 29 :
	float gnss_att_brsght_z_stdv; // Word 30 :
	float gps_position_state_x_stdv; // Word 31 :
	float gps_position_state_y_stdv; // Word 32 :
	float gps_position_state_z_stdv; // Word 33 :
	float gps_pvt_lever_arm_x_stdv; // Word 36 :
	float gps_pvt_lever_arm_y_stdv; // Word 37 :
	float gps_pvt_lever_arm_z_stdv; // Word 38 :
	float gps_position_state_x; // Word 39 :
	float gps_position_state_y; // Word 40 :
	float gps_position_state_z; // Word 41 :
};

#endif // __HGuideAPI_Msg_6424_h__
