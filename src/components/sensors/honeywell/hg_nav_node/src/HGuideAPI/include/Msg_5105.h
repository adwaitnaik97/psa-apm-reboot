#ifndef __HGuideAPI_Msg_5105_h__
#define __HGuideAPI_Msg_5105_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5105 : GALILEO Ephemeris Message
//
// Decoded GLONASS L1 C/A ephemeris
// The message contains the Galileo I/NAV and F/NAV ephemeris information.
// Multiple messages are transmitted, one for each SVID with date. 
// The I/NAV messages are only transmitted on E1 and E5b.
// The F/NAV messages are only transmitted on E5a.
// 
class HGUIDE_DLL Msg_5105
{
public:
	Msg_5105();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 220;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5105; // Message ID
	static const uint32_t MessageLength = 55; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode; // GPS Mode of operation
	
	// type of Galileo Ephemeris
 	// 0 = I/NAV
 	// 1 = F/NAV
	uint16_t ephem_type;
	uint16_t satellite_ID; // Satellite identifier
	uint8_t E5bHealth; // E5b health status bits
	uint8_t E5bDVS; // E5b data validity status
	uint8_t E1bHealth; // E1b health status bits
	uint8_t E1bDVS; // E1b data validity status
	uint8_t E5aHealth; // E5a health status bits
	uint8_t E5aDVS; // E5a data validity status
	uint16_t IODnav; // Issue of data ephemeris
	uint8_t SISA_Index; // Signal in space accuracy (unitless)
	
	// Identifies the source signal:
 	// 0 = Unknown
 	// 1 = E1b
 	// 2 = E5b
 	// 3 = E1b and E5b
	uint8_t INAV_Source;
	uint32_t T0e; // Ephemeris reference time (s)
	uint32_t T0c; // Clock correction data reference time of week from the I/NAV message (s)
	double M0; // Mean anomaly at ref time (radians)
	double DeltaN; // Mean motion difference (radians/s)
	double Ecc; // Eccentricity (unitless)
	double RootA; // Square root of semi-major axis
	double I0; // Inclination angle at ref time (radians)
	double IDot; // Rate of inclination angle (radians/s)
	double Omega0; // Longitude of ascending node of orbital plane at weekly epoch (radians)
	double Omega; // Argument of perigee (radians)
	double OmegaDot; // Rate of right ascension (radians/s)
	double Cuc; // Amplitude of the cosine harmonic correction term to the argument of latitude (radians)
	double Cus; // Amplitude of the sine harmonic correction term to the argument of latitude (radians)
	double Crc; // Amplitude of the cosine harmonic correction term to the orbit radius (m)
	double Crs; // Amplitude of the sine harmonic correction term to the orbit radius (m)
	double Cic; // Amplitude of the cosine harmonic correction term to the angle of inclination (radians)
	double Cis; // Amplitude of the sine harmonic correction term to the angle of inclination (radians)
	double Af0; // SV clock bias correction coefficient from the I/NAV message (s)
	double Af1; // SV clock drift correction coefficient from the I/NAV message (s/s)
	double Af2; // SV clock drift rate correction coefficient from the I/NAV message (s/s^2)
	double E1E5aBGD; // E1, E5a broadcast group delay
	double E1E5bBGD; // E1, E5b broadcast group delay
};

#endif // __HGuideAPI_Msg_5105_h__
