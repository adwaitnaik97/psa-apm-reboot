#ifndef __HGuideAPI_Msg_1401_h__
#define __HGuideAPI_Msg_1401_h__
#pragma once

#include <cstdint>


// 0x1401 : Navigation Input Message for INS
//
// Input Position / Velocity / Attitude to the INS
// Corresponding validity values has to be marked as 1 = valid for the INS to use the value
// sending a STDV <= 0 will result to reset to default value in the INS.
// Read the detailed description below before usage. If unsure, consult with Honeywell.
// 
class HGUIDE_DLL Msg_1401
{
public:
	Msg_1401();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 156;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1401; // Message ID
	static const uint32_t MessageLength = 39; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	bool RequestACKNAKReply; // Request ACK/NAK to this message (0 = false | 1 = true)
	double PositionTov; // [s] data validity time (based on PositionTimeReferenceMode)
	bool PositionValidity; // Validity of the input position (0 = invalid | 1 = valid)
	bool PositionTimeReferenceMode; // PositionTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool PositionCoordinateFrame; // Input Position Coordinate Frame (0 = Latitude/Longitude/Altitude | 1 = ECEF [x,y,z])
	bool PositionStdvValidity; // Validity of the input position STDV (0 = invalid | 1 = valid)
	float Latitude; // [rad] Input Latitude position
	float Longitude; // [rad] Input Longitude position
	float AltitudeHeightAboveEllipsoid; // [m] Input Altitude above ellipsoid
	float EcefPositionX; // [m] Input ECEF X position
	float EcefPositionY; // [m] Input ECEF Y position
	float EcefPositionZ; // [m] Input ECEF Z position
	double VelocityTov; // data validity time (based on VelocityTimeReferenceMode)
	bool VelocityValidity; // Validity of the input velocity (0 = invalid | 1 = valid)
	bool VelocityTimeReferenceMode; // VelocityTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool VelocityCoordinateFrame; // Input Velocity Coordinate Frame (0 = Latitude/Longitude/Altitude | 1 = ECEF [x,y,z])
	bool VelocityStdvValidity; // Validity of the input velocity STDV (0 = invalid | 1 = valid)
	float VelocityNorthOrEcefVelocityX; // [m/s] Input North velocity or ECEF X Velocity
	float VelocityEastOrEcefVelocityY; // [m/s] Input East velocity or ECEF Y Velocity
	float VelocityDownOrEcefVelocityZ; // [m/s] Input Down velocity or ECEF Z Velocity
	double AttitudeTov; // [s] data validity time (based on AttitudeTimeReferenceMode)
	bool AttitudeValidity; // Validity of the input attitude (0 = invalid | 1 = valid)
	bool AttitudeTimeReferenceMode; // AttitudeTov Time of Reference (0 = TOV referenced to GPS Time | 1 = TOV referenced to timestamp of message receipt)
	bool AttitudeCoordinateFrame; // Input Attitude Coordinate Frame (0 = DCM [matrix 3x3] | 1 = Roll/Pitch/Heading)
	bool AttitudeStdvValidity; // Validity of the input Attitude STDV (0 = invalid | 1 = valid)
	float DCM11; // DCM for Input Attitude [xx]
	float DCM12; // DCM for Input Attitude [xy]
	float DCM13; // DCM for Input Attitude [xz]
	float DCM21; // DCM for Input Attitude [yx]
	float DCM22; // DCM for Input Attitude [yy]
	float DCM23; // DCM for Input Attitude [yz]
	float DCM31; // DCM for Input Attitude [zx]
	float DCM32; // DCM for Input Attitude [zy]
	float DCM33; // DCM for Input Attitude [zz]
	float EulerAnglesRoll; // [rad] Input Roll
	float EulerAnglesPitch; // [rad] Input Pitch
	float EulerAnglesTrueHeading; // [rad] Input True Heading
	float PositionStdvNorth; // [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float PositionStdvEast; // [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float PositionStdvDown; // [m] STDV North, East, Down (passing 0 equals default = 100 m)
	float VelocityStdvNorth; // [m/s] STDV North(passing 0 means default = 1 m/s)
	float VelocityStdvEast; // [m/s] STDV East(passing 0 means default = 1 m/s)
	float VelocityStdvDown; // [m/s] STDV Down (passing 0 means default = 1 m/s)
	float EulerAnglesStdvRoll; // [rad] STDV Roll (passing 0 means default = 0.0873 rad)
	float EulerAnglesStdvPitch; // [rad] STDV Pitch (passing 0 means default = 0.0873 rad)
	float EulerAnglesStdvTrueHeading; // [rad] STDV True Heading (passing 0 means default = 0.0873 rad)
};

#endif // __HGuideAPI_Msg_1401_h__
