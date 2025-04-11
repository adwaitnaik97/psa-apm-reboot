#ifndef __HGuideAPI_Msg_6724_h__
#define __HGuideAPI_Msg_6724_h__
#pragma once

#include <cstdint>


// 0x6724 : Current Profile Velocity Data
//
// Current profile velocity data (variable length in Nortek DF3).
// Message outputs data for 4 beams and 30 cells.
// Velocity Scaling is located in 0x6723 as 'velocityScaling' field
// multiply the velBeam value by this scale and store in float32 to use the value.
// 
class HGUIDE_DLL Msg_6724
{
public:
	Msg_6724();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 264;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6724; // Message ID
	static const uint32_t MessageLength = 66; // Message length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // Trigger/ping time of ensemble (bottom track trigger time)
	int16_t velBeam0[30]; // Velocity Beam 0 Cells 0-29 (10^velocity scaling m/s)
	int16_t velBeam1[30]; // Velocity Beam 1 Cells 0-29 (10^velocity scaling m/s)
	int16_t velBeam2[30]; // Velocity Beam 2 Cells 0-29 (10^velocity scaling m/s)
	int16_t velBeam3[30]; // Velocity Beam 3 Cells 0-29 (10^velocity scaling m/s)
};

#endif // __HGuideAPI_Msg_6724_h__
