#ifndef __HGuideAPI_Msg_2402_h__
#define __HGuideAPI_Msg_2402_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2402 : Smoothed Navigation
//
// DO NOT USE, message deprecated, use 0x6403 for position, 0x6405 for attitude and 0x6504 for Velocity data
// 
class HGUIDE_DLL Msg_2402
{
public:
	Msg_2402();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 228;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2402; // Message ID
	static const uint32_t MessageLength = 57; // Message Length
	uint32_t Checksum; // Checksum
	uint32_t InsGnssSummary; // INS/GPS BIT Summary
	ins_mode_table_t INSMode; // INS Mode table
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	double systemTov; // Time since Power Up - Always Valid [sec]
	int32_t gps_week; // [-] gps week number
	int32_t utc_time_figure_of_merit;
	int32_t gps_figure_of_merit;
	int32_t ins_blended_figure_of_merit;
	double PositionTov; // [s] Position time of validity
	double Latitude; // [rad] Latitude position
	double Longitude; // [rad] Longitude position
	double AltitudeHeightAboveEllipsoid; // [m] Altitude Above Ellipsoid
	double AltitudeMeanSeaLevel; // [m] Altitude above Mean Sea Level
	double EcefPositionX; // [m] ECEF coordinates X position
	double EcefPositionY; // [m] ECEF coordinates Y position
	double EcefPositionZ; // [m] ECEF coordinates Z position
	double VelocityTov; // [s] Velocity time of validity
	double NorthVelocity; // [m/s] NED North Velocity
	double EastVelocity; // [m/s] NED East Velocity
	double DownVelocity; // [m/s] NED Down Velocity
	double EcefVelocityX; // [m/s] Velocity in X axis of ECEF Frame
	double EcefVelocityY; // [m/s] Velocity in Y axis of ECEF Frame
	double EcefVelocityZ; // [m/s] Velocity in Z axis of ECEF Frame
	double AttitudeTov; // [s] Attitude time of validity
	float EulerAnglesRoll; // [rad] Roll Angle of Vehicle Body
	float EulerAnglesPitch; // [rad] Pitch Angle of Vehicle Body
	float EulerAnglesTrueHeading; // [rad] Heading Angle of Vehicle Body
	float wander_angle;
	double DCM11; // DCM of Vehicle body to local body [xx]
	double DCM12; // DCM of Vehicle body to local body [xy]
	double DCM13; // DCM of Vehicle body to local body [xz]
	double DCM21; // DCM of Vehicle body to local body [yx]
	double DCM22; // DCM of Vehicle body to local body [yy]
	double DCM23; // DCM of Vehicle body to local body [yz]
	double DCM31; // DCM of Vehicle body to local body [zx]
	double DCM32; // DCM of Vehicle body to local body [zy]
	double DCM33; // DCM of Vehicle body to local body [zz]
	float angular_rate_x; // [rad/s] Angular Rate in Vehicle Body X
	float angular_rate_y; // [rad/s] Angular Rate in Vehicle Body Y
	float angular_rate_z; // [rad/s] Angular Rate in Vehicle Body Z
	float linear_acceleration_x; // [m/s/s] Linear Acceleration in Vehicle Body X
	float linear_acceleration_y; // [m/s/s] Linear Acceleration in Vehicle Body Y
	float linear_acceleration_z; // [m/s/s] Linear Acceleration in Vehicle Body Z
	int32_t attitude_figure_of_merit;
	float q0_vehicle_body_to_ecef; // Vehicle body to ECEF quaternion [s]
	float q1_vehicle_body_to_ecef; // Vehicle body to ECEF quaternion [i]
	float q2_vehicle_body_to_ecef; // Vehicle body to ECEF quaternion [j]
	float q3_vehicle_body_to_ecef; // Vehicle body to ECEF quaternion [k]
};

#endif // __HGuideAPI_Msg_2402_h__
