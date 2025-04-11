#ifndef __HGuideAPI_Msg_1105_h__
#define __HGuideAPI_Msg_1105_h__
#pragma once

#include <cstdint>


// 0x1105 : Input Message for Magnetic Heading
//
// The 0x1105 message can be used to input Magnetic Heading.  These inputs include time and Heading in radians.
// 
class HGUIDE_DLL Msg_1105
{
public:
	Msg_1105();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1105; // Message ID
	static const uint32_t MessageLength = 10; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	double MagneticHeadingTov;
	bool MagneticHeadingValidity; // Validity bit for Magnetic Heading (1 = Valid | 0 = Invalid)
	bool TimeReferenceMode; // MagneticHeadingTov Time of Reference (0 = Referenced to GPS Tov | 1 = Referenced to Message receipt time)
	bool MagneticVariationValidity; // Validity bit for Magnetic Variation (1 = Valid | 0 = Invalid)
	float MagneticHeading; // [rad] Magnetic Heading
	float MagneticVariation; // [mgauss] Magnetic Variation
};

#endif // __HGuideAPI_Msg_1105_h__
