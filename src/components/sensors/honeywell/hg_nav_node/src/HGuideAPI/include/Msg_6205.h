#ifndef __HGuideAPI_Msg_6205_h__
#define __HGuideAPI_Msg_6205_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/event_in_mark_t.h>


// 0x6205 : Vehicle All-In-One Pos, Vel, Att of Event-In
//
// The 0x6205 message provides the Geodectic position, velocity and attitude estimates of the vehicle at the time of the Event-In signal occurred.
// When multiple event in sources are enabled, sources are differentiated by the markport field.
// 
class HGUIDE_DLL Msg_6205
{
public:
	Msg_6205();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 124;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6205; // Message ID
	static const uint32_t MessageLength = 31; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	event_in_mark_t markPort; // Event port the message pertains to
	double system_time_of_event_in; // [s] EventIn System Time of Validity
	double gps_time_of_event_in; // [s] EventIn GPS Time of Week
	double Latitude; // [rad] WGS-84 Latitude
	double Longitude; // [rad] WGS-84 Longitude
	double AltitudeHeightAboveEllipsoid; // [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float LatitudeSTDV; // [rad] Estimated Latitude Error Standard Deviation (1-sigma)
	float LongitudeSTDV; // [rad] Estimated Longitude Error Standard Deviation (1-sigma)
	float AltitudeHeightAboveEllipsoidSTDV; // [m] Estimated Altitude Error Standard Deviation (1-sigma)
	float NorthVelocity; // [m/s] North Velocity
	float EastVelocity; // [m/s] East Velocity
	float DownVelocity; // [m/s] Down Velocity
	float NorthVelocitySTDV; // [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float EastVelocitySTDV; // [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float DownVelocitySTDV; // [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
	float Roll; // [rad] Roll
	float Pitch; // [rad] Pitch
	float Heading; // [rad] Heading
	float EulerAnglesSTDVRoll; // [rad] Estimated Roll Error Standard Deviation
	float EulerAnglesSTDVPitch; // [rad] Estimated Pitch Error Standard Deviation
	float EulerAnglesSTDVHeading; // [rad] Estimated Heading Error Standard Deviation
};

#endif // __HGuideAPI_Msg_6205_h__
