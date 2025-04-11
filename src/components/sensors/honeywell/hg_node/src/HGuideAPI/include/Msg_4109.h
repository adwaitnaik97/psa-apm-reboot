#ifndef __HGuideAPI_Msg_4109_h__
#define __HGuideAPI_Msg_4109_h__
#pragma once

#include <cstdint>


// 0x4109 : Attitude Initialization Message
//
// The message 0x4109 is used to provide GNSS attitude/heading measurements to the HGuide.
// 
class HGUIDE_DLL Msg_4109
{
public:
	Msg_4109();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 60;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4109; // Message ID
	static const uint32_t MessageLength = 15; // Message Length
	uint32_t Checksum; // Checksum
	bool RequestAckNak; // 0 = do not request the ACK/NAK of message being received | 1 = request the ACK/NAK of message being received
	bool RollValid; // Input Roll is valid (1 = valid | 0 = invalid)
	bool PitchValid; // Input Pitch is valid (1 = valid | 0 = invalid)
	bool HeadingValid; // Input Heading is valid (1 = valid | 0 = invalid)
	bool TovMode; // Time of Validity Mode: 0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt
	double MessageTov; // [s] Time of validity based on TovMode
	float Roll; // [rad] Roll
	float Pitch; // [rad] Pitch
	float TrueHdg; // [rad] Heading
	float RollStdv; // [rad] Roll standard deviation
	float PitchStdv; // [rad] Pitch standard deviation
	float TrueHdgStdv; // [rad] Heading standard deviation
	float Baseline; // [m] Baseline between both antennas (distance from RF1 to RF2 antenna)
};

#endif // __HGuideAPI_Msg_4109_h__
