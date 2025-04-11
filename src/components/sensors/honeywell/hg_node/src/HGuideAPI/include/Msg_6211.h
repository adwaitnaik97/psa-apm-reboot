#ifndef __HGuideAPI_Msg_6211_h__
#define __HGuideAPI_Msg_6211_h__
#pragma once

#include <cstdint>

#include <include/event_out_mark_t.h>


// 0x6211 : Time Mark of Event-Out
//
// The 0x6211 message displays the time of validity of the Event-Out signal.
// 
class HGUIDE_DLL Msg_6211
{
public:
	Msg_6211();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x6211; // Message ID
	static const uint32_t MessageLength = 10; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	event_out_mark_t markPort; // Event-Out port the message pertains to
	double systemTov; // [sec] INS system time of Event-Out
	double gpsTov; // [sec] GPS Time of Event-Out in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // GPS Week of Event-Out [#]
};

#endif // __HGuideAPI_Msg_6211_h__
