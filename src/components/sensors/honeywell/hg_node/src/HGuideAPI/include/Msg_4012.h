#ifndef __HGuideAPI_Msg_4012_h__
#define __HGuideAPI_Msg_4012_h__
#pragma once

#include <cstdint>

// Navigation Config on Restart
struct restart_nav_config_t
{
	bool gyro_bias; // RESERVED
	bool accel_bias; // RESERVED
	bool gyro_sf; // RESERVED
	bool accel_sf; // RESERVED
	bool main_ant_lev; // Store the current, calculated lever arm for the main antenna
	bool antenna_boresight; // Store the current, calculated boresight quaternion between the main and aux antenna
	bool nav_init; // Store the current POSE for nav to start with on next cycle

	void Default()
	{
		gyro_bias = 0;
		accel_bias = 0;
		gyro_sf = 0;
		accel_sf = 0;
		main_ant_lev = 0;
		antenna_boresight = 0;
		nav_init = 0;
	}
};


// 0x4012 : Command Power Cycle Nav Initialization
//
// Send position, attitude, and velocity (assumed zero) information to unit.  
// Information used on next power cycle only to initialize the unit into navigation.  
// Read the detailed description below before usage. If unsure, consult with Honeywell.
// 
class HGUIDE_DLL Msg_4012
{
public:
	Msg_4012();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4012; // Message ID
	static const uint32_t MessageLength = 16; // Message Length
	uint32_t Checksum; // Checksum
	restart_nav_config_t nav_init_config;
	uint32_t Sigma_Latitude; // RESERVED
	uint32_t Sigma_Longitude; // RESERVED
	uint32_t Sigma_Altitude; // RESERVED
	uint32_t Sigma_Heading; // RESERVED
};

#endif // __HGuideAPI_Msg_4012_h__
