#ifndef __HGuideAPI_Msg_6725_h__
#define __HGuideAPI_Msg_6725_h__
#pragma once

#include <cstdint>


// 0x6725 : Current Profile Amplitude Data
//
// Current profile amplitude data (variable length in Nortek DF3).
// Message outputs data for 4 beams and 30 cells.
// 
class HGUIDE_DLL Msg_6725
{
public:
	Msg_6725();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 144;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6725; // Message ID
	static const uint32_t MessageLength = 36; // Message length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // Trigger/ping time of ensemble (bottom track trigger time)
	uint8_t ampBeam0[30]; // Amplitude Beam 0 Cells 0-29 (1 count)
	uint8_t ampBeam1[30]; // Amplitude Beam 1 Cells 0-29 (1 count)
	uint8_t ampBeam2[30]; // Amplitude Beam 2 Cells 0-29 (1 count)
	uint8_t ampBeam3[30]; // Amplitude Beam 3 Cells 0-29 (1 count)
};

#endif // __HGuideAPI_Msg_6725_h__
