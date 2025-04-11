#ifndef __HGuideAPI_Msg_2428_h__
#define __HGuideAPI_Msg_2428_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2428 : GNSS (PVT, PR/DR) Measurement Normalized Residuals
//
// Residuals of INS PVT and Pseudorange / Deltarange measurements
// 
class HGUIDE_DLL Msg_2428
{
public:
	Msg_2428();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 180;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message F
	static const uint32_t MessageId = 0x2428; // Message ID
	static const uint32_t MessageLength = 45; // Message Length in
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float GNSS_Position_X; // [sigma]
	float GNSS_Position_Y; // [sigma]
	float GNSS_Position_Z; // [sigma]
	float GNSS_Velocity_X; // [sigma]
	float GNSS_Velocity_Y; // [sigma]
	float GNSS_Velocity_Z; // [sigma]
	uint32_t Number_of_PR_channels;
	float GPS_PR_Channel_1; // [sigma]
	float GPS_PR_Channel_2; // [sigma]
	float GPS_PR_Channel_3; // [sigma]
	float GPS_PR_Channel_4; // [sigma]
	float GPS_PR_Channel_5; // [sigma]
	float GPS_PR_Channel_6; // [sigma]
	float GPS_PR_Channel_7; // [sigma]
	float GPS_PR_Channel_8; // [sigma]
	float GPS_PR_Channel_9; // [sigma]
	float GPS_PR_Channel_10; // [sigma]
	float GPS_PR_Channel_11; // [sigma]
	float GPS_PR_Channel_12; // [sigma]
	uint32_t Number_of_DR_channels;
	float GPS_DR_Channel_1; // [sigma]
	float GPS_DR_Channel_2; // [sigma]
	float GPS_DR_Channel_3; // [sigma]
	float GPS_DR_Channel_4; // [sigma]
	float GPS_DR_Channel_5; // [sigma]
	float GPS_DR_Channel_6; // [sigma]
	float GPS_DR_Channel_7; // [sigma]
	float GPS_DR_Channel_8; // [sigma]
	float GPS_DR_Channel_9; // [sigma]
	float GPS_DR_Channel_10; // [sigma]
	float GPS_DR_Channel_11; // [sigma]
	float GPS_DR_Channel_12; // [sigma]
};

#endif // __HGuideAPI_Msg_2428_h__
