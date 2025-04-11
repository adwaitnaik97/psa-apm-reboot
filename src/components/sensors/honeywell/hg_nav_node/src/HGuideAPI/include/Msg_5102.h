#ifndef __HGuideAPI_Msg_5102_h__
#define __HGuideAPI_Msg_5102_h__
#pragma once

#include <cstdint>

#include <include/constellation_enum_t.h>
#include <include/gps_mode_table_t.h>

// Single Observation used in the GNSS Range message
struct satellite_pos_t
{
	constellation_enum_t system; // Satellite system
	uint32_t satelliteID; // Satellite ID Number
	double pos_x; // Satellite X co-ordinates (ECEF,m)
	double pos_y; // Satellite Y co-ordinates (ECEF,m)
	double pos_z; // Satellite Z co-ordinates (ECEF,m)
	double clk_corr; // Satellite clock correction (m)
	double iono_delay; // Ionosphere delay (m)
	double tropo_delay; // Troposphere delay (m)

	void Default()
	{
		system = GPS;
		satelliteID = 0;
		pos_x = 0;
		pos_y = 0;
		pos_z = 0;
		clk_corr = 0;
		iono_delay = 0;
		tropo_delay = 0;
	}
};


// 0x5102 : GNSS Satellite Position
//
// the 0x5102 message contains Satellite positions in ECEF Cartesian coordinates.
// When combined with a 0x5101 GNSS RANGE message, this data set contains the decoded satellite information necessary to compute the solution
// The length of the message is based on the amount of satellites. Check the num_satellites field to properly decode the message.
// This message can be taken as Satellite Position input by any HGuide INS system.
// 
class HGUIDE_DLL Msg_5102
{
public:
	Msg_5102();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return (40+num_satellites*18)*4;}
	//Set Repeated variable max size
	bool set_sat_position_size(uint32_t size);
	//Get current valid size of the variable (either max size or current size)
	uint32_t get_sat_position_size(void);

private:
	uint32_t max_sat_position_size = 0;

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x5102; // Message ID
	
	// Message Length [Number of 32-bit Words]
	// Make sure to recalculate based on the amount satellite positions
	uint32_t MessageLength;
	uint32_t Checksum; // 32-bit CRC
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	gps_mode_table_t GPSMode; // GPS Mode of operation
	uint32_t num_satellites; // Number of satellites to follow - defines the size of the variable size message
	
	// Satellite positions - each 18 words long
	// Repeated block - make sure to allocate memory for the sat_position via .set_sat_position_size(size) method
	// if amount of data is higher than the allocated size, only allocated size shall be populated
	satellite_pos_t * sat_position;
};

#endif // __HGuideAPI_Msg_5102_h__
