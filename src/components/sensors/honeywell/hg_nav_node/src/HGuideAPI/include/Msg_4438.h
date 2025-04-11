#ifndef __HGuideAPI_Msg_4438_h__
#define __HGuideAPI_Msg_4438_h__
#pragma once

#include <cstdint>


// 0x4438 : DMI configuration message
//
// The message 0x4438 is used to configure the device for a specific odometer.
//      -  The lever arms must be measured be in the vehicle frame.
//      -  The vehicle frame must be in line with the odometer +Direction of Travel.
//      -  The lever arm distance is from the vehicle frame origin to the wheel center.
//      -  The odometer should be on the rear wheel of a four wheel vehicle.
//      -  Ideally, the HGuide device should be mounted centrally on the rear axles.
//      -  Lever arms are read by the navigation filter upon power up.  
//      -  Cycle power to ensure that the navigation filter is properly initialized.
//      -  Direct Quadrature input shares the HW lines with COM1 RS422 port and will disable it.
// The default values for parameters listed in words 44 through 72 have been well tested with a
// 3600 pules per revolution quadrature rotary encoder sensor. Do not change. 
// 
class HGUIDE_DLL Msg_4438
{
public:
	Msg_4438();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 96;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4438; // Message ID
	static const uint32_t MessageLength = 24; // Message Length
	uint32_t Checksum; // Checksum
	
	// Enable the Direct Quadrature (DMI) Input (0 = Disabled (Default) | 1 = Enabled)
 	// NOTE: Enabling the Quadrature Decoder will disable the COM1 RS422 port
	bool QdecEnable;
	
	// [m] Distance per pulse from external Quadrature Encoder.
	// This value is used by the direct quadrature (DMI) input
	// QdecDistancePerPulse = wheelCircumference / numberOfPulsesPerRevolution
	float QdecDistancePerPulse;
	float OdometerLeverArmX; // [m] X Lever Arm from Vehicle Frame
	float OdometerLeverArmY; // [m] Y Lever Arm from Vehicle Frame
	float OdometerLeverArmZ; // [m] Z Lever Arm from Vehicle Frame
	float OdometerMeasurementNoise; // [m/sample] Expected noise in measurement (STDV)
	float OdometerTheshold; // [sigma] Rejection sigma
	float OdometerInitialScaleFactorUncertainty; // [0..1] Initial Scale Factor Uncertainty
	float OdometerScaleFactorNoise; // [1/sqrt(hz)] Expected Scale Factor Noise
	float OdometerYawBoresightStdv; // [rad] Heading Error STDV
	float OdometerYawBoresightProcessNoise; // [rad/sqrt(hz)]
	float OdometerInitialPitchBoresightUncertainty; // [rad] Pitch Error STDV
	float OdometerInitialPitchBoresightProcessNoise; // [rad/sqrt(hz)]
	bool ChangeOdometerLeverArm; // 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
};

#endif // __HGuideAPI_Msg_4438_h__
