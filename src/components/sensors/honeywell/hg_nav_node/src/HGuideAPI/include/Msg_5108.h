#ifndef __HGuideAPI_Msg_5108_h__
#define __HGuideAPI_Msg_5108_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5108 : GNSS Receiver PVT Measurement Output
//
// Position, Velocty and Time information from GNSS receiver. The position is refered to Main (RF1) antenna
// This message can be taken as an PVT input by any HGuide INS system.
// 
class HGUIDE_DLL Msg_5108
{
public:
	Msg_5108();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 128;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5108; // Message ID
	static const uint32_t MessageLength = 32; // Message length
	uint32_t Checksum; // Checksum
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // GPS Week
	gps_mode_table_t GPSMode; // GPS Mode of operation
	double Latitude; // [rad] WGS-84 Latitude
	double Longitude; // [rad] WGS-84 Longitude
	double AltitudeHeightAboveEllipsoid; // [m] Altitude : Height Above Ellipsoid
	float NorthVelocity; // [m/s] North Velocity
	float EastVelocity; // [m/s] East Velocity
	float DownVelocity; // [m/s] Down Velocity
	float RxClkBias; // [m] Receiver Clock Bias
	
	// Altitude type:
 	// 0 = Undefined
 	// 1 = Geoid Altitude
 	// 2 = Geoid Height
 	// 3 = Ellipsoid WGS-84
	uint8_t Datum;
	uint8_t NumberOfSvs; // Number of SVs
	uint32_t CorrInfo; // RESERVED for details about corrections used
	uint32_t SignalInfo; // RESERVED detailed signal info`
	uint32_t PPPInfo; // RESERVED PPP corrections info
	float LatitudeSTDV; // [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float LongitudeSTDV; // [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float AltitudeHeightAboveEllipsoidSTDV; // [m] Estimated Altitude Error Standard Deviation (1-sigma)
	float NorthVelocitySTDV; // [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float EastVelocitySTDV; // [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float DownVelocitySTDV; // [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
	double systemTov; // [sec] System Time of Validity
	float PDOP; // Position dilution of precision (PDOP) [ DOP from 0.0 to 10.2]
	float HDOP; // Horizontal Position dilution of precision (HDOP) [ DOP from 0.0 to 10.2]
	float VDOP; // Vertical Position dilution of precision (VDOP) [ DOP from 0.0 to 10.2]
	float TDOP; // Time dilution of precision (TDOP) [ DOP from 0.0 to 10.2]
	float GeoidSeparation; // [m] Separation of geoid from ellipsoid
};

#endif // __HGuideAPI_Msg_5108_h__
