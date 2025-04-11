#ifndef __HGuideAPI_Msg_4202_h__
#define __HGuideAPI_Msg_4202_h__
#pragma once

#include <cstdint>

#include <include/event_out_mark_t.h>


// 0x4202 : Event-Out Control Input Message
//
// The 0x4202 message is used to enable and disable output Events as well as configure their parameters
// 
class HGUIDE_DLL Msg_4202
{
public:
	Msg_4202();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 48;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4202; // Message ID
	static const uint32_t MessageLength = 12; // Message Length
	uint32_t Checksum; // Checksum
	event_out_mark_t MarkPort; // Event-Out Port Number
	
	// Edge Trigger:
	// 0 = Rising Edge (Active High)
	// 1 = Falling Edge (Active Low)
	// 2 = Both rising and falling Edge (Alternating High-Low)
	uint8_t Polarity;
	bool Enable_6211_Time; // Output 0x6211 Time Stamp + GPS Week
	bool ChangeEventOutSetUp; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	
	// Event-Out Trigger
 	// 0 = Disabled
 	// 1 = Frequency (100Hz Max)
 	// 2 = Distance Travelled
 	// 3 = Velocity Change (Reserved)
 	// 4 = Attitude Change (Reserved)
 	// 5 = Position Change (Reserved)
 	// 6 = Acceleration Threshold (Reserved)
 	// 7 = Angular Rate Threshold (Reserved)
	uint8_t Trigger;
	
	// Value to trigger Event-Out. The unit is different for each trigger source.
 	// Consult the Trigger table to learn more
	float Trigger_Value;
	float time_offset; // Time of Event Output = Trigger of event + time_offset [seconds]
	
	// Event Out pulse width
 	// in case of Polarity = 2 - Minimum time to elapse before allowing another switch
	float pulse_width;
};

#endif // __HGuideAPI_Msg_4202_h__
