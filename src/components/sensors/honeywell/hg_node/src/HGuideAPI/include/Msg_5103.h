#ifndef __HGuideAPI_Msg_5103_h__
#define __HGuideAPI_Msg_5103_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5103 : GPS Ephemeris Message
//
// Decoded GPS L1 C/A ephemerides
// This message contains a single set of GPS ephemeris parameters.
// 
class HGUIDE_DLL Msg_5103
{
public:
	Msg_5103();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 264;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5103; // Message ID
	static const uint32_t MessageLength = 66; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode; // GPS Mode of operation
	uint32_t prn_number; // Satellite PRN number
	double tow; // Time stamp of subframe 1 (s)
	
	// Code(s) on L2 Channel
 	// 00 = Reserved,
 	// 01 = P code ON,
 	// 10 = C/A code ON.
 	// RINEX Code on L2 = code_l2 + 0.5
	uint8_t code_l2;
	uint8_t l2_p_data; // flag indicating the NAV data stream was commanded OFF on the P-code of the L2 channel (0 = ON | 1 = OFF)
	uint8_t health; // Health status - a 6-bit health code as defined in IS-GPS-200
	uint32_t IODE1; // Issue of ephemeris data 1
	uint32_t IODE2; // Issue of ephemeris data 2
	uint32_t week; // toe week number (computed from Z count week)
	uint32_t z_week; // Z count week number. This is the week number from subframe 1 of the ephemeris. The ‘toe week’ (field #7) is derived from this to account for rollover
	double toe; // Reference time for ephemeris (s)
	double A; // Semi-major axis (m)
	double dN; // (ΔN)Mean motion difference (radians/s)
	double M0; // Mean anomaly of reference time (radians)
	double ecc; // Eccentricity, dimensionless
	double w; // (ω) Argument of perigee (radians)
	double cuc; // Amplitude of cosine harmonic correction term to the argument of latitude (radians)
	double cus; // Amplitude of sine harmonic correction term to the argument of latitude (radians)
	double crc; // Amplitude of cosine harmonic correction term to the orbit radius (m)
	double crs; // Amplitude of sine harmonic correction term to the orbit radius (m)
	double cic; // Amplitude of cosine harmonic correction term to the angle of inclination (radians)
	double cis; // Amplitude of sine harmonic correction term to the angle of inclination (radians)
	double Inclination_angle; // Inclination angle at reference time (radians)
	double Inclination_rate; // Rate of inclination angle (radians/s)
	double wo; // (wo) Right ascension (radians)
	double wi; // (ώ)Rate of right ascension (radians/s)
	uint32_t iodc; // Issue of data clock
	double toc; // SV clock correction term (s)
	double tgd; // Estimated group delay difference (s)
	double af0; // Clock aging parameter (s)
	double af1; // Clock aging parameter (s/s)
	double af2; // Clock aging parameter (s/s/s)
	
	// Anti-spoofing on:
 	// 0 = FALSE
 	// 1 = TRUE
	bool AS;
	
	// Corrected mean motion (radians/s)
 	// This field is computed by the receiver.
	double N;
	
	// User Range Accuracy variance (m^2)
 	// The ICD specifies that the URA index transmitted in the ephemerides can be converted to a nominal
 	// standard deviation value using an algorithm listed there. We publish the square of the nominal value (variance).
 	// The correspondence between the original URA index and the value output is shown in Table: URA Variance
	double URA;
};

#endif // __HGuideAPI_Msg_5103_h__
