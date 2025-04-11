#ifndef __HGuideAPI_Msg_4201_h__
#define __HGuideAPI_Msg_4201_h__
#pragma once

#include <cstdint>

#include <include/event_in_mark_t.h>


// 0x4201 : Event-In Control Input Message
//
// The 0x4201 message is used to enable and disable output (0x62xx) messages in response to
// to an analog Event In signal or to an internal PPS signal. 
// 
class HGUIDE_DLL Msg_4201
{
public:
	Msg_4201();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 28;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4201; // Message ID
	static const uint32_t MessageLength = 7; // Message Length
	uint32_t Checksum; // Checksum
	event_in_mark_t MarkPort; // Event-In Port Number
	bool Enable_6201_Time; // Output 0x6201 Time Stamp + GPS Week
	bool Enable_6202_Position; // Output 0x6202 Position and Time Stamp
	bool Enable_6203_Velocity; // Output 0x6203 Velocity and Time Stamp
	bool Enable_6204_Attitude; // Output 0x6204 Attitude and Time Stamp
	bool Enable_6205_Full_State; // Output 0x6205 Position, Velocity, Attitude and Time Stamp
	bool ChangeEventInSetUp; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	
	// Edge Trigger:
	// 0 = Rising Edge
	// 1 = Falling Edge
	// 2 = Both rising and falling Edge
	uint8_t Polarity;
	float time_offset; // Time of Validity of Event = System Time of event + time_offset [seconds]
	
	// Minimum time to elapse before allowing another input [seconds]
	// It is recommended to use the time_guard value if you expect noise in your trigger signal.
	// This circumvents the probability of unwanted triggers.
	float time_guard;
};

#endif // __HGuideAPI_Msg_4201_h__
