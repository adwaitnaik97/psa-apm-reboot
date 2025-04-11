#ifndef __HGuideAPI_Msg_6108_h__
#define __HGuideAPI_Msg_6108_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6108 : GNSS Receiver PVT Measurement Output
//
// RAW position from GNSS receiver. The position is refered to Main (RF1) antenna
// This message is being outputted only when GNSS Signal is avaiable
// 
class HGUIDE_DLL Msg_6108
{
public:
	Msg_6108();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 128;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6108; // Message ID
	static const uint32_t MessageLength = 32; // Message length
	uint32_t Checksum; // Checksum
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // GPS Week
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	double Latitude; // [rad] WGS-84 Latitude
	double Longitude; // [rad] WGS-84 Longitude
	double AltitudeHeightAboveEllipsoid; // [m] Altitude : Height Above Ellipsoid
	float NorthVelocity; // [m/s] North Velocity
	float EastVelocity; // [m/s] East Velocity
	float DownVelocity; // [m/s] Down Velocity
	float RxClkBias; // RESERVED - [m] Receiver Clock Bias
	
	// Altitude type:
 	// 0 = Undefined
 	// 1 = Geoid Altitude
 	// 2 = Geoid Height
 	// 3 = Ellipsoid WGS-84
	uint8_t Datum;
	uint8_t TimeReference; // RESERVED - Time Reference
	uint8_t NumberOfSvs; // Number of SVs
	uint8_t RTKfixProg; // RESERVED
	uint32_t CorrInfo; // RESERVED
	uint32_t SignalInfo; // RESERVED
	uint32_t PPPInfo; // RESERVED
	float LatitudeSTDV; // [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float LongitudeSTDV; // [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float AltitudeHeightAboveEllipsoidSTDV; // [m] Estimated Altitude Error Standard Deviation (1-sigma)
	float NorthVelocitySTDV; // [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float EastVelocitySTDV; // [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float DownVelocitySTDV; // [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
	double systemTov; // [sec] System Time of Validity
	float PDOP; // RESERVED - Position dilution of precision (PDOP) [ DOP from 0.0 to 10.2]
	float HDOP; // RESERVED - Horizontal Position dilution of precision (HDOP) [ DOP from 0.0 to 10.2]
	float VDOP; // RESERVED - Vertical Position dilution of precision (VDOP) [ DOP from 0.0 to 10.2]
	float TDOP; // RESERVED - Time dilution of precision (TDOP) [ DOP from 0.0 to 10.2]
	float Undulation; // [m] Undulation - the difference between the geoid and the ellipsoid
};

#endif // __HGuideAPI_Msg_6108_h__
