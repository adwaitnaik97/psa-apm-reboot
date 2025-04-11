#ifndef __HGuideAPI_Msg_2201_h__
#define __HGuideAPI_Msg_2201_h__
#pragma once

#include <cstdint>

// Custom type to define time validity and PPS mode
struct time_validity_bits_t
{
	bool gpsTovValid; // GPS Time Validity (0 = Invalid | 1 = Valid)
	bool utcTimeValid; // UTC Time Validity (0 = Invalid | 1 = Valid)
	uint8_t PPS_Mode; // PPS Mode of operation (0 = Disabled | 1 = Connected to Event IN | 4 = Connected to System Time)

	void Default()
	{
		gpsTovValid = 0;
		utcTimeValid = 0;
		PPS_Mode = 0;
	}
};


// 0x2201 : Time Mark PPS
//
// DO NOT USE for Event IN timing - use 0x620x messages
// 
class HGUIDE_DLL Msg_2201
{
public:
	Msg_2201();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 96;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2201; // Message ID
	static const uint32_t MessageLength = 24; // Message Length
	uint32_t Checksum; // Checksum
	time_validity_bits_t time_validity_bits; // validity of time information and PPS mode
	
	// 0 = External Source
	// 1 = accuracy <= 1ns
	// 2 = accuracy <= 10ns
	// 3 = accuracy <= 100ns
	// 4 = accuracy <= 1us
	// 5 = accuracy <= 10us
	// 6 = accuracy <= 100us
	// 7 = accuracy <= 1ms
	// 8 = accuracy <= 10ms
	// 9 = Unknown/Fault
	int32_t utc_time_figure_of_merit;
	double systemTov; // [sec] Time since Power Up - Always Valid
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32_t utc_day_month_year_date;
	uint8_t utc_day_of_week; // day of week (0 = Sunday | ... | 6 = Saturday)
	uint16_t utc_day_of_year; // day of year (1 - 366 | 0 = Invalid)
	uint8_t utc_day_of_month; // day in month (1 - 31 | 0 = Invalid)
	uint8_t utc_month; // month number (1 - 12 | 0 = Invalid)
	uint16_t utc_year; // year number (0 = Invalid | >= 1980)
	uint32_t utc_hour_min_sec;
	uint8_t utc_second; // 0 - 60 (60 is used when leap second is added )
	uint8_t utc_minute; // 0 - 59
	uint8_t utc_hour; // 0 - 23
	double utc_time; // UTC Time within day
	double ins_gps_internal_system_time_pps_in; // [s] INS time of event input
	double gps_time_pps_in; // [s] GPS Time of event input
	uint32_t pps_in_count; // [-] Event in count
};

#endif // __HGuideAPI_Msg_2201_h__
