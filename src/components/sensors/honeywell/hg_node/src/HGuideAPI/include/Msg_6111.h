#ifndef __HGuideAPI_Msg_6111_h__
#define __HGuideAPI_Msg_6111_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>


// 0x6111 : Results of Motion detection algorithm
//
// Results of 5 tests needed for motion detection algorithm (automatic Zero Velocity and Heading Change => ZUPT).
// Test 1 Rotation.
// Test 2 Speed.
// Test 3 Angular Rate.
// Test 4 Linear Acceleration.
// Test 5 Odometer Data.
// Settling time is indicating how are those test passed
// 
class HGUIDE_DLL Msg_6111
{
public:
	Msg_6111();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 160;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6111; // Message ID
	static const uint32_t MessageLength = 40; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	double Latitude; // [rad] Latitude of ZUPT detection
	double Longitude; // [rad] Longitude of ZUPT detection
	float Test1_RotationX; // [rad/s] X axis (Roll) Value of rotation
	float Test1_RotationY; // [rad/s] Y axis (Pitch) Value of rotation
	float Test1_RotationZ; // [rad/s] Z axis (Yaw) Value of rotation
	float Test1_RotationNormRate; // [rad/s] Actual value of sum of Angular Rates in all axes
	float Test1_RotationNormRateThreshold; // [rad/s] Threshold of sum of rotation in all axes
	float Test2_SpeedValid; // Speed Valid
	float Test2_Speed; // [m/s] Actual sum of speed in all axes
	float Test2_SpeedThreshold; // [m/s] Threshold of sum of speed in all axes
	float Test3_AngularRateInstantX; // [rad/s] X Angular Rate Filtered by Instant FN 3dB filter
	float Test3_AngularRateInstantY; // [rad/s] Y Angular Rate Filtered by Instant FN 3dB filter
	float Test3_AngularRateInstantZ; // [rad/s] Z Angular Rate Filtered by Instant FN 3dB filter
	float Test3_InstantFilterBandwidth; // [Hz] Instant Filter Bandwidth for exit of ZUPT
	float Test3_AngularRateNominalX; // [rad/s] X Angular Rate Filtered by Nominal FN 3dB filter
	float Test3_AngularRateNominalY; // [rad/s] Y Angular Rate Filtered by Nominal FN 3dB filter
	float Test3_AngularRateNominalZ; // [rad/s] Z Angular Rate Filtered by Nominal FN 3dB filter
	float Test3_NominalFilterBandwidth; // [Hz] Nominal Filter Bandwidth for entering of ZUPT
	float Test3_AngularRateX; // [rad/s] X Actual Angular rate
	float Test3_AngularRateY; // [rad/s] Y Actual Angular rate
	float Test3_AngularRateZ; // [rad/s] Z Actual Angular rate
	float Test3_AngularRateThreshold; // [rad/s] Threshold of sum of angular rate in all axes
	float Test4_LinearAcceleration; // [m/s2] Actual value of sum of linar accelerations in all axes
	float Test4_LinearAccelerationThreshold; // [m/s2] Threshold of sum of linear accelerations in all axes
	float Test5_OdometerDeltaDistance; // [m] Change in distance based on Odometer Input
	float Test5_OdometerDeltaDistanceThreshold; // [m] Threshold of change in distance based on Odometer Input
	float SettlingTime_Odometer; // [s] Number of seconds when Odometer sent no pulses
	float SettlingTime_Stationary; // [s] Amount of time stationary
	float SettlingTime_Threshold; // [s] Threshold of stationary times`
};

#endif // __HGuideAPI_Msg_6111_h__
