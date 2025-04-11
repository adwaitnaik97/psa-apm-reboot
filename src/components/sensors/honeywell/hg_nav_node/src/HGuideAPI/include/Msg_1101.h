#ifndef __HGuideAPI_Msg_1101_h__
#define __HGuideAPI_Msg_1101_h__
#pragma once

#include <cstdint>


// 0x1101 : Input Message for Barometric Altitude
//
// The 0x1101 message can be used to input barometric altitude.  These inputs include time and altitude.
// 
class HGUIDE_DLL Msg_1101
{
public:
	Msg_1101();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 40;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1101; // Message ID
	static const uint32_t MessageLength = 10; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	double BarometricAltitudeTov; // [s] Time of Validity for Barometric Altitude
	bool BarometricAltitudeValidity; // Validity bit for Barometric Altitude (1 = Valid | 0 = Invalid)
	bool TimeReferenceMode; // BarometricAltitudeTov Time of Reference (0 = Referenced to GPS Tov | 1 = Referenced to Message receipt time)
	float BarometricAltitudeMSLGeoid; // [m] Send Barometric Altitude (MSL Geoid)
};

#endif // __HGuideAPI_Msg_1101_h__
