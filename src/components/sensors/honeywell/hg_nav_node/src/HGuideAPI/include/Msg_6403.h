#ifndef __HGuideAPI_Msg_6403_h__
#define __HGuideAPI_Msg_6403_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6403 : INS Geodetic Position
//
// This is the basic source of position from the INS system.
// Message contains Geodetic Position (Latitude, Longitude, Altitude) with associated Standard deviations
// The position is referenced from Vehicle Frame
// 
class HGUIDE_DLL Msg_6403
{
public:
	Msg_6403();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 72;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6403; // Message ID
	static const uint32_t MessageLength = 18; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	double Latitude; // [rad] WGS-84 Latitude
	double Longitude; // [rad] WGS-84 Longitude
	double AltitudeHeightAboveEllipsoid; // [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Summary Word
	float LatitudeSTDV; // [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float LongitudeSTDV; // [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float AltitudeHeightAboveEllipsoidSTDV; // [m] Estimated Altitude Error Standard Deviation (1-sigma)
};

#endif // __HGuideAPI_Msg_6403_h__
