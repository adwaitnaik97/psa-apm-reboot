#ifndef __HGuideAPI_Msg_6202_h__
#define __HGuideAPI_Msg_6202_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/event_in_mark_t.h>


// 0x6202 : Vehicle Geodetic Position of Event-In
//
// The 0x6202 message provides the geodectic position estimate of the vehicle at the time of the Event-In signal occurred.
// When multiple event in sources are enabled, sources are differentiated by the markport field.
// 
class HGUIDE_DLL Msg_6202
{
public:
	Msg_6202();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 76;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6202; // Message ID
	static const uint32_t MessageLength = 19; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	event_in_mark_t markPort; // Event port the message pertains to
	double system_time_of_event_in; // [s] System Time of Validity of EventIn
	double gps_time_of_event_in; // [s] GPS Time of Week of EventIn
	double Latitude; // [rad] WGS-84 Latitude
	double Longitude; // [rad] WGS-84 Longitude
	double AltitudeHeightAboveEllipsoid; // [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float LatitudeSTDV; // [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float LongitudeSTDV; // [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float AltitudeHeightAboveEllipsoidSTDV; // [m] Estimated Altitude Error Standard Deviation (1-sigma)
};

#endif // __HGuideAPI_Msg_6202_h__
