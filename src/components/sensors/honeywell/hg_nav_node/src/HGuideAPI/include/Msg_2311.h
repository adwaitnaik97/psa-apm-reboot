#ifndef __HGuideAPI_Msg_2311_h__
#define __HGuideAPI_Msg_2311_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x2311 : Unfiltered Inertial Data
//
// This is the basic source of inertial data from the INS system.
// Inertial data is output in the IMU body reference frame. See the 0x2001 Case to Nav Lever Arms and Quaternion for details.
// Log this message and 0x2001 along with output from GNSS receiver (COM2) for post-processing
// 
class HGUIDE_DLL Msg_2311
{
public:
	Msg_2311();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 68;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2311; // Message ID
	static const uint32_t MessageLength = 17; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary; // INS/GPS BIT Summary
	double delta_theta_x; // [rad] Delta Theta X
	double delta_theta_y; // [rad] Delta Theta Y
	double delta_theta_z; // [rad] Delta Theta Z
	int32_t gps_week; // [-] Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	double delta_velocity_x; // [m/s] Delta Velocity X
	double delta_velocity_y; // [m/s] Delta Velocity Y
	double delta_velocity_z; // [m/s] Delta Velocity Z
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
};

#endif // __HGuideAPI_Msg_2311_h__
