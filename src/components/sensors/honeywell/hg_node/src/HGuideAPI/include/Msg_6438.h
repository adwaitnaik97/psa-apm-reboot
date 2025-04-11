#ifndef __HGuideAPI_Msg_6438_h__
#define __HGuideAPI_Msg_6438_h__
#pragma once

#include <cstdint>


// 0x6438 : Filter Calibration Of Odometer Inputs
//
// Odometer Calibration information. Notably Scale factor, lever arms and residuals
// 
class HGUIDE_DLL Msg_6438
{
public:
	Msg_6438();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 192;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6438; // Message ID
	static const uint32_t MessageLength = 48; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [s] Time since INS power up
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // [-] GPS week no.
	float residual_x; // [m] X axis filter measurement residual
	float residual_y; // [m] Y axis filter measurement residual
	float residual_z; // [m] Z axis filter measurement residual
	float scalefactor_correction; // [<0; 1>] Correction of Scalefactor (1 = 100% correction)
	float scalefactor_correction_pitchsens; // DO NOT USE [<0; 1>] Correction of Scalefactor (1 = 100% correction) sensed from pitch
	float scalefactor_correction_pitchratesens; // DO NOT USE [<0; 1>] Correction of Scalefactor (1 = 100% correction) sensed from pitch rate
	float boresight_yaw; // [rad] Odometer Boresight Yaw with respect to Navigation Frame
	float boresight_yaw_rollsens; // DO NOT USE [rad] Odometer Boresight Yaw with respect to Navigation Frame sensed from roll
	float boresight_yaw_rocksens; // DO NOT USE [rad] Odometer Boresight Yaw with respect to Navigation Frame sensed from rock
	float boresight_pitch; // [rad] Odometer Boresight Pitch with respect to Navigation Frame
	float boresight_pitch_rollsens; // DO NOT USE [rad] Odometer Boresight Pitch with respect to Navigation Frame as sensed from roll
	float boresight_pitch_rocksens; // DO NOT USE [rad] Odometer Boresight Pitch with respect to Navigation Frame as sensed from rock
	float lever_arm_x; // [m] X odometer estimated lever arm
	float lever_arm_y; // [m] Y odometer estimated lever arm
	float lever_arm_z; // [m] Z odometer estimated lever arm
	float scalefactor_correction_stdv; // [<0; 1>] Standard deviation of Scalefactor correction
	float scalefactor_correction_pitchsens_stdv; // DO NOT USE [<0; 1>] Standard deviation of Scalefactor correction
	float scalefactor_correction_pitchratesens_stdv; // DO NOT USE [<0; 1>] Standard deviation of Scalefactor correction
	float boresight_yaw_stdv; // [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame
	float boresight_yaw_rollsens_stdv; // DO NOT USE [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame sensed from roll
	float boresight_yaw_rocksens_stdv; // DO NOT USE [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame sensed from rock
	float boresight_pitch_stdv; // [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame
	float boresight_pitch_rollsens_stdv; // DO NOT USE [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame sensed from roll
	float boresight_pitch_rocksens_stdv; // DO NOT USE [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame sensed from rock
	float lever_arm_stdv_x; // [m] X standard deviations of Odometer position
	float lever_arm_stdv_y; // [m] Y standard deviations of Odometer position
	float lever_arm_stdv_z; // [m] Z standard deviations of Odometer position
	float stored_lever_arm_x; // [m] X Initially Stored position of Odometer with respect to Navigation Frame
	float stored_lever_arm_y; // [m] Y Initially Stored position of Odometer with respect to Navigation Frame
	float stored_lever_arm_z; // [m] Z Initially Stored position of Odometer with respect to Navigation Frame
	float boresight_roll; // [rad] Odometer Boresight Roll with respect to Navigation Frame
	float boresight_stdv_roll; // [rad] Odometer Boresight Roll standard deviation with respect to Navigation Frame
};

#endif // __HGuideAPI_Msg_6438_h__
