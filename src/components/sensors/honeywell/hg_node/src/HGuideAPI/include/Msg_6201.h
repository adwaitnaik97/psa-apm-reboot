#ifndef __HGuideAPI_Msg_6201_h__
#define __HGuideAPI_Msg_6201_h__
#pragma once

#include <cstdint>

#include <include/event_in_mark_t.h>


// 0x6201 : Time Mark of Event-In
//
// The 0x6201 message displays the time of validity of the Event-In signal.
// This message is used by Inertial Explorer conversion utilities to compute the solution in Event-In times.
// Note that the time of validity is in all event in response messages.
// When multiple event in sources are enabled, sources are differentiated by the markport field.
// 
class HGUIDE_DLL Msg_6201
{
public:
	Msg_6201();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6201; // Message ID
	static const uint32_t MessageLength = 10; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	event_in_mark_t markPort; // Event port the message pertains to
	double system_time_of_event_in; // [sec] INS system time of EventIn [seconds]
	double gps_time_of_event_in; // [sec] GPS Time of EventIn in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week_of_event_in; // GPS Week of EventIn [#]
};

#endif // __HGuideAPI_Msg_6201_h__
