#ifndef __HGuideAPI_Msg_4401_h__
#define __HGuideAPI_Msg_4401_h__
#pragma once

#include <cstdint>

#include <include/coordinate_frame_t.h>


// 0x4401 : Position, Velocity, Attitude Input Message
//
// Input Position / Velocity / Attitude to the System. The `input_coordinate_frame` enumeration specifies the referenced frame.
// Corresponding validity values has to be marked as 1 = valid for the System to use the value
// sending a STDV <= 0 will result to reset to default value.
// 
class HGUIDE_DLL Msg_4401
{
public:
	Msg_4401();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 160;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4401; // Message ID
	static const uint32_t MessageLength = 40; // Message Length [Number of 32-bit Words]
	uint32_t Checksum; // 32-bit CRC
	bool RequestACKNAKReply; // Request ACK/NAK to this message (0 = false | 1 = true)
	
	// If 1, then time of validity of the data will be obtained from the message time of reception.
 	// If 0, then the GPS time in the message will be used as the time of validity.
	bool TOV_Mode;
	coordinate_frame_t input_coordinate_frame; // Position is referenced to the selected frame
	double solutionTov; // [s] data validity time (based on TOV_Mode)
	bool PositionValidity; // 1 = Use Input Position | 0 = Do not use position fields
	double Latitude; // [rad] Input Latitude position (WGS-84)
	double Longitude; // [rad] Input Longitude position (WGS-84)
	double Altitude; // [m] Input Altitude above ellipsoid (WGS-84)
	bool VelocityValidity; // 1 = Use Input Velocity | 0 = Do not use velocity fields
	double NorthVelocity; // [m/s] Input North velocity
	double EastVelocity; // [m/s] Input East velocity
	double DownVelocity; // [m/s] Input Down velocity
	bool AttitudeValidity; // 1 = Use Input Attitude | 0 = Do not use attitude fields
	double Roll; // [rad] Input Roll
	double Pitch; // [rad] Input Pitch
	double TrueHeading; // [rad] Input True Heading
	float PositionStdvNorth; // [m] STDV North, East, Down (passing 0 equals default)
	float PositionStdvEast; // [m] STDV North, East, Down (passing 0 equals default)
	float PositionStdvDown; // [m] STDV North, East, Down (passing 0 equals default)
	float VelocityStdvNorth; // [m/s] STDV North(passing 0 means default)
	float VelocityStdvEast; // [m/s] STDV East(passing 0 means default)
	float VelocityStdvDown; // [m/s] STDV Down (passing 0 means default)
	float AttitudeStdvRoll; // [rad] STDV Roll (passing 0 means default)
	float AttitudeStdvPitch; // [rad] STDV Pitch (passing 0 means default)
	float AttitudeStdvTrueHeading; // [rad] STDV True Heading (passing 0 means default)
};

#endif // __HGuideAPI_Msg_4401_h__
