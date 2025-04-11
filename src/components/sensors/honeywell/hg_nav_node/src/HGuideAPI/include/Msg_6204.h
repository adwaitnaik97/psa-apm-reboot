#ifndef __HGuideAPI_Msg_6204_h__
#define __HGuideAPI_Msg_6204_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/event_in_mark_t.h>


// 0x6204 : Vehicle Euler Attitudes of Event-In
//
// The 0x6204 message provides the attitude estimate of the vehicle at the time of the Event-In signal occurred.
// When multiple event in sources are enabled, sources are differentiated by the markport field.
// 
class HGUIDE_DLL Msg_6204
{
public:
	Msg_6204();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6204; // Message ID
	static const uint32_t MessageLength = 16; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	event_in_mark_t markPort; // Event port the message pertains to
	double system_time_of_event_in; // [s] System Time of Validity Event In
	double gps_time_of_event_in; // [s] GPS Time of Week Event In
	float Roll; // [rad] Roll
	float Pitch; // [rad] Pitch
	float Heading; // [rad] Heading
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float EulerAnglesSTDVRoll; // [rad] Estimated Roll Error Standard Deviation
	float EulerAnglesSTDVPitch; // [rad] Estimated Pitch Error Standard Deviation
	float EulerAnglesSTDVHeading; // [rad] Estimated Heading Error Standard Deviation
};

#endif // __HGuideAPI_Msg_6204_h__
