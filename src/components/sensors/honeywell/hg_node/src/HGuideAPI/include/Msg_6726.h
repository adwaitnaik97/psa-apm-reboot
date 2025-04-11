#ifndef __HGuideAPI_Msg_6726_h__
#define __HGuideAPI_Msg_6726_h__
#pragma once

#include <cstdint>


// 0x6726 : Current Profile Correlation Data
//
// Current profile correlation data (variable length in Nortek DF3).
// Message outputs data for 4 beams and 30 cells.
// 
class HGUIDE_DLL Msg_6726
{
public:
	Msg_6726();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 144;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6726; // Message ID
	static const uint32_t MessageLength = 36; // Message length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // Trigger/ping time of ensemble (bottom track trigger time)
	uint8_t corBeam0[30]; // Correlation Beam 0 Cells 0-29 [0 – 100]
	uint8_t corBeam1[30]; // Correlation Beam 1 Cells 0-29 [0 – 100]
	uint8_t corBeam2[30]; // Correlation Beam 2 Cells 0-29 [0 – 100]
	uint8_t corBeam3[30]; // Correlation Beam 3 Cells 0-29 [0 – 100]
};

#endif // __HGuideAPI_Msg_6726_h__
