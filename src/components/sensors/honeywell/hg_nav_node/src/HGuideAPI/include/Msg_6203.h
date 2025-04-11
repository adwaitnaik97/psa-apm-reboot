#ifndef __HGuideAPI_Msg_6203_h__
#define __HGuideAPI_Msg_6203_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/event_in_mark_t.h>


// 0x6203 : Vehicle NED Velocity of Event-In
//
// The 0x6203 message provides the North,East,Down velocity estimate of the vehicle at the time of the Event-In signal occurred.
// When multiple event in sources are enabled, sources are differentiated by the markport field.
// 
class HGUIDE_DLL Msg_6203
{
public:
	Msg_6203();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6203; // Message ID
	static const uint32_t MessageLength = 16; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	event_in_mark_t markPort; // Event port the message pertains to
	double system_time_of_event_in; // [s] System Time of Validity Event In
	double gps_time_of_event_in; // [s] GPS Time of Week Event In
	float NorthVelocity; // [m/s] North Velocity
	float EastVelocity; // [m/s] East Velocity
	float DownVelocity; // [m/s] Down Velocity
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Mode Summary
	float NorthVelocitySTDV; // [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float EastVelocitySTDV; // [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float DownVelocitySTDV; // [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
};

#endif // __HGuideAPI_Msg_6203_h__
