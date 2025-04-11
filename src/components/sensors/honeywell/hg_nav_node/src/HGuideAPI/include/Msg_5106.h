#ifndef __HGuideAPI_Msg_5106_h__
#define __HGuideAPI_Msg_5106_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5106 : BEIDOU Ephemeris Message
//
// Decoded BDS2 and BDS3 ephemeris
// This message contains a single set of BDS2/BSD3 ephemeris parameters with appropriate scaling applied. 
// Multiple messages are transmitted, one for each SV ephemeris collected.
// 
class HGUIDE_DLL Msg_5106
{
public:
	Msg_5106();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 292;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5106; // Message ID
	static const uint32_t MessageLength = 73; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode; // GPS Mode of operation
	uint8_t ephem_type; // Type of BeiDou ephemeris (0 = BDS2 | 1 = BDS3)
	
	// BeiDou Satellite type
	// 0 = Reserved
	// 1 = GEO
	// 2 = IGSO
	// 3 = MEO
	uint8_t sat_type;
	uint32_t satellite_ID; // ID/ranging code
	uint32_t Week; // BeiDou week number
	double URA; // User range accuracy (m). This is the evaluated URAI/URA lookup-table value.
	uint32_t health1; // Autonomous satellite health flag. 0 means broadcasting satellite is good and 1 means not.
	double tgd1; // Equipment group delay differential for the B1 signal (s)
	double tgd2; // Equipment group delay differential for the B2 signal (s)
	uint32_t AODC; // Age of data, clock
	double tgdb2cp; // [BDS3] Group delay differential between the B1C pilot component and B3I signal(sent by B2a and B1c)
	double tgdb2bi; // [BDS3] Group delay differential between the b2b and B3i (s),only sent by B2b non-GEO satellites
	uint32_t IODC; // [BDS3] Age of data, clock
	uint32_t toc; // Reference time of clock parameters (s)
	double a0; // Constant term of clock correction polynomial (s)
	double a1; // Linear term of clock correction polynomial (s/s)
	double a2; // Quadratic term of clock correction polynomial (s/s^2)
	uint32_t AODE; // Age of data, ephemeris
	uint32_t toe; // Reference time of ephemeris parameters (s)
	double RootA; // Square root of semi-major axis (sqrt(m))
	double ecc; // Eccentricity (dimensionless)
	double w; // (ω) Argument of perigee (radians)
	double dN; // (ΔN) Mean motion difference from computed value (radians/s)
	double M0; // Mean anomaly at reference time (radians)
	double omega0; // (Ω0) Longitude of ascending node of orbital of plane computed according to reference time (radians)
	double omegai; // (Ω`) Rate of right ascension (radians/s)
	double i0; // Inclination angle at reference time (radians)
	double IDOT; // Rate of inclination angle (radians/second)
	double cuc; // Amplitude of cosine harmonic correction term to the argument of latitude (radians)
	double cus; // Amplitude of sine harmonic correction term to the argument of latitude (radians)
	double crc; // Amplitude of cosine harmonic correction term to the orbit radius (m)
	double crs; // Amplitude of sine harmonic correction term to the orbit radius (m)
	double cic; // Amplitude of cosine harmonic correction term to the angle of inclination (radians)
	double cis; // Amplitude of sine harmonic correction term to the angle of inclination (radians)
	double tgd3; // [BDS3] Tgdb2ap,group delay differential between the B2a pilot component and the B3I signal(sent by both B2a and B1c)
	double IscB1cd; // [BDS3] Group delay differential between the B1C data and pilot components,only sent by B1c
	double IscB2ad; // [BDS3] Group delay differential between the B2a data and pilot components,only sent by B2a
	uint32_t SISMAI; // [BDS3] RESERVED - signal in space monitoring accuracy index for BDS3 satellites, will be published in future ICD
	double deltaA; // [BDS3] semi-major axis difference at reference time
	double Adot; // [BDS3] change rate in semi-major axis
	double deltaN0Dot; // [BDS3] Rate of mean motion difference form computed value at reference time
};

#endif // __HGuideAPI_Msg_5106_h__
