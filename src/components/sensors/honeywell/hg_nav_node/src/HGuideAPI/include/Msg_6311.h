#ifndef __HGuideAPI_Msg_6311_h__
#define __HGuideAPI_Msg_6311_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/coordinate_frame_t.h>


// 0x6311 : Unfiltered Inertial Data User Selected Reference Frame
//
// Inertial data output in a user defined reference frame.
// 
class HGUIDE_DLL Msg_6311
{
public:
	Msg_6311();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 72;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6311; // Message ID
	static const uint32_t MessageLength = 18; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary; // INS/GPS BIT Summary
	double delta_theta_x; // [rad] Delta Theta X
	double delta_theta_y; // [rad] Delta Theta Y
	double delta_theta_z; // [rad] Delta Theta Z
	int32_t gps_week; // GPS Week
	double delta_velocity_x; // [m/s] Delta Velocity X
	double delta_velocity_y; // [m/s] Delta Velocity Y
	double delta_velocity_z; // [m/s] Delta Velocity Z
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	coordinate_frame_t current_coordinate_frame; // Current Coordinate Frame
};

#endif // __HGuideAPI_Msg_6311_h__
