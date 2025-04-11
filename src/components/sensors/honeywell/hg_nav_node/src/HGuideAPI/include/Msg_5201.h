#ifndef __HGuideAPI_Msg_5201_h__
#define __HGuideAPI_Msg_5201_h__
#pragma once

#include <cstdint>


// 0x5201 : PPS GNSS Time
//
// GNSS Receiver outputs this message to specify accurate time of PPS output.
// The message can be used by the Navigation device to associate a GPS and UTC time with an Incoming hardware PPS. 
// The message must be received <0.25 seconds after generation of a 1 Hz rising edge positive polarity pulse on the PPS hardware line.
// 
class HGUIDE_DLL Msg_5201
{
public:
	Msg_5201();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 72;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x5201; // Message ID
	static const uint32_t MessageLength = 18; // Message Length
	uint32_t Checksum; // Checksum
	bool gpsTovValid; // is gps time valid
	bool gpsToUTCofsetValid; // is offset between gps and utc time valid
	double gps_time_to_utc_offset; // utc_time = Fmod(gps_time + gps_time_to_utc_offset,86400);
	
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
	double systemTov; // [sec] Time since Power Up - Always Valid when the rising edge of the PPS occurs
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC) when the rising edge of the PPS occurs
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint8_t utc_day_of_week; // day of week (0 = Sunday | ... | 6 = Saturday)
	uint16_t utc_day_of_year; // day of year (1 - 366 | 0 = Invalid)
	uint8_t utc_day_of_month; // day in month (1 - 31 | 0 = Invalid)
	uint8_t utc_month; // month number (1 - 12 | 0 = Invalid)
	uint16_t utc_year; // year number (0 = Invalid | >= 1980)
	uint8_t utc_second; // 0 - 60 (60 is used when leap second is added )
	uint8_t utc_minute; // 0 - 59
	uint8_t utc_hour; // 0 - 23
};

#endif // __HGuideAPI_Msg_5201_h__
