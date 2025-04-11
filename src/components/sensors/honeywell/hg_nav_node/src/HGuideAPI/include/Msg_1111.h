#ifndef __HGuideAPI_Msg_1111_h__
#define __HGuideAPI_Msg_1111_h__
#pragma once

#include <cstdint>

#include <include/trajectory_aiding_validity_discretes_t.h>


// 0x1111 : ESpace Trajectory Aiding
//
// Message is used to aid the GNSS receiver to acquire lock faster
// 
class HGUIDE_DLL Msg_1111
{
public:
	Msg_1111();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 120;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1111; // Message ID
	static const uint32_t MessageLength = 30; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	trajectory_aiding_validity_discretes_t trajectory_aiding_validity; // Select which aiding values are valid
	double ESpaceTrajectoryTOV; // [s] Message Time of validity
	double Latitude; // [rad] Input Latitude
	double Longitude; // [rad] Input Longitude
	double Altitude_height_above_ellipsoid; // [m] Input Altitude above ellipsoid
	double Baro_altitude; // [m] Input barometer altitude
	double True_air_speed; // [m/s] Input True Air speed
	double velocity_north; // [m/s] Input North Velocity
	double velocity_east; // [m/s] Input East Velocity
	double velocity_down; // [m/s] Input Down Velocity
	double velocity_x; // [m/s] Input ECEF Velocity X
	double velocity_y; // [m/s] Input ECEF Velocity Y
	double velocity_z; // [m/s] Input ECEF Velocity Z
	float attitude_roll; // [rad] Input Roll Attitude
	float attitude_pitch; // [rad] Input Pitch Attitude
	float attitude_heading; // [rad] Input Heading Attitude
};

#endif // __HGuideAPI_Msg_1111_h__
