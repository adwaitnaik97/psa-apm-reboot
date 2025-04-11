#ifndef __HGuideAPI_Msg_4738_h__
#define __HGuideAPI_Msg_4738_h__
#pragma once

#include <cstdint>


// 0x4738 : DVL Configuration Initialization
//
// Message to input DVL KF configuration and beam leverarm and rotation.
// 
class HGUIDE_DLL Msg_4738
{
public:
	Msg_4738();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 144;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4738; // Message ID
	static const uint32_t MessageLength = 36; // Message length in 32-bit words
	uint32_t Checksum; // Checksum
	float measNoise; // [m] Measurement noise - stdv of noise on DVL used by KF
	float measRejectThreshold; // [sigma] Measurement rejection threshold - rejection threshold for DVL measurements used by KF
	float sfUncertainty; // [parts] Scale Factor uncertainty - estimated error from DVL scale factor (from datasheet)
	float sfTimeConst; // [hours] Scale Factor Time Constant - time at which the scale factor uncertainty degrades by 1 sigma
	float misalignInitUncertainty; // [mrad] Misalignment initial uncertainty - estimated initial error in the yaw direction of the mounting error relative to the unit
	float misalignTimeConst; // [hours] Misalignment Time Constant - time at which the misalignment uncertainty degrades by 1 sigma
	float pitchBoresightInitUncertainty; // [rad] Pitch boresight initial uncertainty - estimated initial error in the pitch direction of the mounting error relative to the unit
	float pitchBoresightStateNoise; // [rad/sqrt(hz)] Pitch boresight state noise - estimated in run noise in the pitch direction of the mounting error relative to the unit
	float leverarmBeam0[3]; // [m] X, Y, Z Leverarm Beam 0 - Lever Arm from Vehicle Body to DVL Beam 0 in the vehicle frame
	float psiRotationBeam0; // [deg] PSI rotation beam 0 - Angular Rotation From Case Body to Beam 0
	float leverarmBeam1[3]; // [m] X, Y, Z Leverarm Beam 1 - Lever Arm from Vehicle Body to DVL Beam 1 in the vehicle frame
	float psiRotationBeam1; // [deg] PSI rotation beam 1 - Angular Rotation From Case Body to Beam 1
	float leverarmBeam2[3]; // [m] X, Y, Z Leverarm Beam 2 - Lever Arm from Vehicle Body to DVL Beam 2 in the vehicle frame
	float psiRotationBeam2; // [deg] PSI rotation beam 2 - Angular Rotation From Case Body to Beam 2
	float leverarmBeam3[3]; // [m] X, Y, Z Leverarm Beam 3 - Lever Arm from Vehicle Body to DVL Beam 3 in the vehicle frame
	float psiRotationBeam3; // [deg] PSI rotation beam 3 - Angular Rotation From Case Body to Beam 3
	float psiThtRotation; // [deg] PSI THT rotation - Angular Rotation About The Case Y Axis From Case Body to Beam set
	float psiOffset; // [deg] PSI Offset - Angular Rotation About the Case Z Axis From Case Body to Beam set
	bool saveCfgToFlash; // Save configuration to flash (bit 0 of word)
};

#endif // __HGuideAPI_Msg_4738_h__
