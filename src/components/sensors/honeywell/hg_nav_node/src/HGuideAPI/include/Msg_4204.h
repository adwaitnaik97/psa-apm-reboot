#ifndef __HGuideAPI_Msg_4204_h__
#define __HGuideAPI_Msg_4204_h__
#pragma once

#include <cstdint>


// 0x4204 : Antenna lever arms settings
//
// The message 0x4204 is used to provide Antenna Lever arms to the HGuide.
// Lever arms are read upon power up.  After changing lever arms, cycle power.
// Measure from the case frame origin to the antenna phase center.
// The GNSS true heading reference is defined by the projection of RF1 (Main) 
// to RF2 (Aux) on to local level.
// 
class HGUIDE_DLL Msg_4204
{
public:
	Msg_4204();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 52;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4204; // Message ID
	static const uint32_t MessageLength = 13; // Message Length
	uint32_t Checksum; // Checksum
	float RF1AntennaX; // [m] X RF1 (main) antenna lever arm
	float RF1AntennaY; // [m] Y RF1 (main) antenna lever arm
	float RF1AntennaZ; // [m] Z RF1 (main) antenna lever arm
	float RF2AntennaX; // [m] X RF2 (aux) antenna lever arm
	float RF2AntennaY; // [m] Y RF2 (aux) antenna lever arm
	float RF2AntennaZ; // [m] Z RF2 (aux) antenna lever arm
	float RF1AntennaSTDV; // [m] Zero is not valid and STDV will default to 1 Meter
	float RF2AntennaSTDV; // [m] Zero is not valid and STDV will default to 1 Meter
	bool ChangeRF1LeverArm; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF2LeverArm; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF1STDV; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	bool ChangeRF2STDV; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
};

#endif // __HGuideAPI_Msg_4204_h__
