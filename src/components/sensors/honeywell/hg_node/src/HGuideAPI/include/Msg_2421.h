#ifndef __HGuideAPI_Msg_2421_h__
#define __HGuideAPI_Msg_2421_h__
#pragma once

#include <cstdint>

// Status of GNSS aiding
struct gps_aiding_status_t
{
	uint8_t channel_1; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_2; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_3; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_4; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_5; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_6; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_7; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_8; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_9; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_10; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_11; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t channel_12; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t TimeBiasRepartitionEvent; // 0 = No Event Detected | 1 = Event Detected

	void Default()
	{
		channel_1 = 0;
		channel_2 = 0;
		channel_3 = 0;
		channel_4 = 0;
		channel_5 = 0;
		channel_6 = 0;
		channel_7 = 0;
		channel_8 = 0;
		channel_9 = 0;
		channel_10 = 0;
		channel_11 = 0;
		channel_12 = 0;
		TimeBiasRepartitionEvent = 0;
	}
};

// Transfer Align aiding status
struct ta_aiding_status_t
{
	uint8_t position_ecef_X; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t position_ecef_Y; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t position_ecef_Z; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_ecef_X; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_ecef_Y; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_ecef_Z; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t attitude_roll; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t attitude_pitch; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t attitude_heading; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed

	void Default()
	{
		position_ecef_X = 0;
		position_ecef_Y = 0;
		position_ecef_Z = 0;
		velocity_ecef_X = 0;
		velocity_ecef_Y = 0;
		velocity_ecef_Z = 0;
		attitude_roll = 0;
		attitude_pitch = 0;
		attitude_heading = 0;
	}
};

// GNSS PVT aiding status
struct gnss_pvt_aiding_status_t
{
	uint8_t latitude; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t longitude; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t altitude; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_north; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_east; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t velocity_down; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed

	void Default()
	{
		latitude = 0;
		longitude = 0;
		altitude = 0;
		velocity_north = 0;
		velocity_east = 0;
		velocity_down = 0;
	}
};

// Miscelaneous aiding status
struct misc_aiding_status_t
{
	uint8_t zero_velocity_x; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t zero_velocity_y; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t zero_velocity_z; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t magnetic_heading; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t zero_heading_change; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed
	uint8_t barometric_altitude; // 0 = Source Disabled | 1 = Invalid | 2 = Rejected | 3 = Successfully Processed

	void Default()
	{
		zero_velocity_x = 0;
		zero_velocity_y = 0;
		zero_velocity_z = 0;
		magnetic_heading = 0;
		zero_heading_change = 0;
		barometric_altitude = 0;
	}
};


// 0x2421 : Filter measurement status
//
// status of all available and used filter measurements.
// 
class HGUIDE_DLL Msg_2421
{
public:
	Msg_2421();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 124;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2421; // Message ID
	static const uint32_t MessageLength = 31; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	gps_aiding_status_t gps_pr_aiding_status; // Pseudo range aiding status
	gps_aiding_status_t gps_dr_aiding_status; // Delta range aiding status
	ta_aiding_status_t ta_aiding_status; // Position Aiding, Velocity Aiding, Attitude Aiding
	gnss_pvt_aiding_status_t gnss_pvt_aiding_status; // GNSS Position / Velocity
	misc_aiding_status_t misc_aiding_status; // Zero Velocity , Magnetic Heading , Static Heading, Baro Altimeter
	uint32_t Available_Measurements; // Number of available measurements.
	uint32_t Successfully_Processed; // Number of successfully processed measurements.
	uint32_t Failed_Processing; // Number of failed measurements.
};

#endif // __HGuideAPI_Msg_2421_h__
