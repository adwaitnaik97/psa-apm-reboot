#ifndef __HGuideAPI_Msg_2021_h__
#define __HGuideAPI_Msg_2021_h__
#pragma once

#include <cstdint>


// 0x2021 : INS Initialization Info
//
// Reporting of the INS intialization data
// 
class HGUIDE_DLL Msg_2021
{
public:
	Msg_2021();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 224;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2021; // Message ID
	static const uint32_t MessageLength = 56; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32_t initialization_status_1;
	uint32_t initialization_status_2;
	uint32_t number_tm_pps_pulses_rcvd;
	uint32_t number_time_msgs_rcvd;
	uint32_t clock_calibration_status;
	double oscillator_drift;
	uint32_t almanac_database;
	uint32_t ephemeris_database;
	uint32_t iono_database;
	uint32_t subframe_4_database;
	uint32_t subframe_5_database;
};

#endif // __HGuideAPI_Msg_2021_h__
