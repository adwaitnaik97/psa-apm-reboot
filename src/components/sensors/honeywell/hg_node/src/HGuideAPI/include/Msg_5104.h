#ifndef __HGuideAPI_Msg_5104_h__
#define __HGuideAPI_Msg_5104_h__
#pragma once

#include <cstdint>

#include <include/gps_mode_table_t.h>


// 0x5104 : GLONASS Ephemeris Message
//
// Decoded GLONASS L1 C/A ephemeris
// This message contains GLONASS L1 C/A ephemeris information. GLONASS ephemerides are referenced to the PZ90.02 geodetic datum. 
// No adjustment between the GPS and GLONASS reference frames are made for positioning. 
// Multiple messages are transmitted, one for each SVID with data.
// 
class HGUIDE_DLL Msg_5104
{
public:
	Msg_5104();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 188;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x5104; // Message ID
	static const uint32_t MessageLength = 47; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode; // GPS Mode of operation
	
	// Slot information offset - PRN identification
	// (Slot + 37). This is also called SLOTO in Connect
	uint16_t slot_offset;
	uint16_t freq_offset; // Frequency channel offset for satellite in the range 0 to 20
	
	// Satellite type where:
	// 0 = GLO_SAT
	// 1 = GLO_SAT_M (M type)
	// 2 = GLO_SAT_K (K type)
	uint8_t sat_type;
	uint16_t e_week; // Reference week of ephemeris (GPS reference time)
	uint32_t e_time; // Reference time of ephemeris (GPS reference time) (ms)
	int32_t t_offset; // Integer seconds between GPS and GLONASS time. A positive value implies GLONASS is ahead of GPS reference time.
	uint16_t Nt; // Calendar number of day within 4 year interval starting at Jan 1 of a leap year
	uint32_t issue; // 15 minute interval number corresponding to ephemeris reference time
	
	// Ephemeris health where:
	// 0-3 = GOOD
	// 4-15 = BAD
	uint32_t health1;
	double pos_x; // X coordinate for satellite at reference time (PZ-90.02) (m)
	double pos_y; // Y coordinate for satellite at reference time (PZ-90.02) (m)
	double pos_z; // Z coordinate for satellite at reference time (PZ-90.02) (m)
	double vel_x; // X coordinate for satellite velocity at reference time (PZ-90.02) (m/s)
	double vel_y; // Y coordinate for satellite velocity at reference time (PZ-90.02) (m/s)
	double vel_z; // Z coordinate for satellite velocity at reference time (PZ-90.02), (m/s)
	double LS_acc_x; // X coordinate for lunisolar acceleration at reference time (PZ-90.02), (m/s/s)
	double LS_acc_y; // Y coordinate for lunisolar acceleration at reference time (PZ-90.02) (m/s/s)
	double LS_acc_z; // Z coordinate for lunisolar acceleration at reference time (PZ-90.02) (m/s/s)
	double tau_n; // Correction to the nth satellite time t_n relative to GLONASS time t_c (s)
	double delta_tau_n; // Time difference between navigation RF signal transmitted in L2 sub-band and navigation RF signal transmitted in L1 sub-band by nth satellite (s)
	double gamma; // Frequency correction (s/s)
	uint32_t Tk; // Time of frame start (since start of GLONASS day) (s)
	uint32_t P; // Technological parameter
	uint32_t Ft; // User range
	uint32_t age; // Age of data (days)
	uint32_t Flags; // Information flags, see Table: GLONASS Ephemeris Flags Coding
	
	// P1 Flag - Time interval between adjacent values of (tb) parameter in minutes
 	// 00b (0) = 0 minutes
 	// 01b (1) = 30 minutes
 	// 10b (2) = 45 minutes
 	// 11b (3) = 60 minutes
	uint8_t flag_p1;
	
	// P2 Flag - Flag of oddness or evenness of the value of (tb) (for intervals of 30 or 60 minutes)
	// 0 = even
	// 1 = odd
	bool flag_p2;
	
	// P3 Flag - Number of satellites with almanac information within current subframe
	// 0 = four
	// 1 = five
	bool flag_p3;
	
	// P4 Flag - Flag to show that ephemeris parameters are present. "1" indicates that updated ephemeris or frequency/time parameters have been uploaded by the control
	// 0 = no updated ephemeris
	// 1 = updated ephemeris present
	bool flag_p4;
};

#endif // __HGuideAPI_Msg_5104_h__
