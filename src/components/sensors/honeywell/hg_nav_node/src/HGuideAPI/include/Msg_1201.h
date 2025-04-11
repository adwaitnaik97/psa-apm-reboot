#ifndef __HGuideAPI_Msg_1201_h__
#define __HGuideAPI_Msg_1201_h__
#pragma once

#include <cstdint>


// 0x1201 : Time Mark PPS Input
//
// This is used by the Navigation device to associate a GPS and UTC time with an
// Incoming hardware PPS.  The message must be received <0.25 seconds after generation
// of a 1 Hz rising edge positive polarity pulse on the PPS hardware line.
// 
class HGUIDE_DLL Msg_1201
{
public:
	Msg_1201();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1201; // Message ID
	static const uint32_t MessageLength = 15; // Message Length
	uint32_t Checksum; // Checksum
	bool gpsTovValid; // is gps time valid
	bool gpsToUTCofsetValid; // is offset between gps and utc time valid
	double gps_time_to_utc_offset; // utc_time = Fmod(gps_time + gps_time_to_utc_offset,86400);
	double PPSgpsTov; // [sec] GPS time within current week when the rising edge of the PPS occurs
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
};

#endif // __HGuideAPI_Msg_1201_h__
