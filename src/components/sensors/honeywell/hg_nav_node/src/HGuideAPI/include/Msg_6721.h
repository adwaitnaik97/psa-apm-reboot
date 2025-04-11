#ifndef __HGuideAPI_Msg_6721_h__
#define __HGuideAPI_Msg_6721_h__
#pragma once

#include <cstdint>

#include <include/dvl_track_status_t.h>


// 0x6721 : Nortek DVL Bottom Track Data
//
// Output of Bottom Track Data from Nortek DVL.
// Reference Nortek DVL binary data format DF21.
// 
class HGUIDE_DLL Msg_6721
{
public:
	Msg_6721();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 236;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6721; // Message ID
	uint32_t Checksum; // Checksum
	static const uint32_t MessageLength = 59; // Message length in 32-bit word
	double systemTov; // [s] Time of validity - trigger/ping time of message ensemble
	uint32_t serialNumber; // DVL Serial Number
	uint8_t version; // DVL Version
	uint8_t offsetOfData; // NOT APPLICABLE to this message. Number of bytes from start of data from Nortek DVL msg format.
	uint8_t year; // Years since 1900
	uint8_t month; // Month, zero based (Jan = 0, Feb = 1, etc.)
	uint8_t day; // Day
	uint8_t hour; // [hr] Hour
	uint8_t minute; // [min] Minute
	uint8_t seconds; // [sec] Seconds
	uint16_t microseconds; // [usec] Microseconds (raw value in 100 usec)
	uint16_t numOfBeams; // Number of beams used [0 - 4 beams used]
	uint32_t error; // DVL Error number
	dvl_track_status_t status; // DVL Status
	float speedOfSound; // [m/s] Speed of sound
	float temperature; // [deg C] Water Temperature
	float pressure; // [Bar] Pressure
	float velBeam0; // Velocity of beam 0 in m/s [invalid set to -32.768 m/s]
	float velBeam1; // Velocity of beam 1 in m/s [invalid set to -32.768 m/s]
	float velBeam2; // Velocity of beam 2 in m/s [invalid set to -32.768 m/s]
	float velBeam3; // Velocity of beam 3 in m/s [invalid set to -32.768 m/s]
	float distBeam0; // Vertical distance beam 0 in meters [invalid set to 0.0 m]
	float distBeam1; // Vertical distance beam 1 in meters [invalid set to 0.0 m]
	float distBeam2; // Vertical distance beam 2 in meters [invalid set to 0.0 m]
	float distBeam3; // Vertical distance beam 3 in meters [invalid set to 0.0 m]
	float fomBeam0; // Figure of merit beam 0 in m/s [invalid set to 10.0 m/s]
	float fomBeam1; // Figure of merit beam 1 in m/s [invalid set to 10.0 m/s]
	float fomBeam2; // Figure of merit beam 2 in m/s [invalid set to 10.0 m/s]
	float fomBeam3; // Figure of merit beam 3 in m/s [invalid set to 10.0 m/s]
	float dt1Beam0; // Time diff 1 beam 0 in seconds
	float dt1Beam1; // Time diff 1 beam 1 in seconds
	float dt1Beam2; // Time diff 1 beam 2 in seconds
	float dt1Beam3; // Time diff 1 beam 3 in seconds
	float dt2Beam0; // Time diff 2 beam 0 in seconds
	float dt2Beam1; // Time diff 2 beam 1 in seconds
	float dt2Beam2; // Time diff 2 beam 2 in seconds
	float dt2Beam3; // Time diff 2 beam 3 in seconds
	float timeVelEstBeam0; // Duration of velocity estimate for beam 0 in seconds
	float timeVelEstBeam1; // Duration of velocity estimate for beam 1 in seconds
	float timeVelEstBeam2; // Duration of velocity estimate for beam 2 in seconds
	float timeVelEstBeam3; // Duration of velocity estimate for beam 3 in seconds
	float velX; // Velocity X in m/s [invalid set to -32.768 m/s]
	float velY; // Velocity Y in m/s [invalid set to -32.768 m/s]
	float velZ1; // Velocity Z1 in m/s [invalid set to -32.768 m/s]
	float velZ2; // Velocity Z2 in m/s [invalid set to -32.768 m/s]
	float fomX; // Figure of merit in m/s [invalid set to 10.0 m/s]
	float fomY; // Figure of merit in m/s [invalid set to 10.0 m/s]
	float fomZ1; // Figure of merit in m/s [invalid set to 10.0 m/s]
	float fomZ2; // Figure of merit in m/s [invalid set to 10.0 m/s]
	float dt1X; // Time diff 1 in X in seconds
	float dt1Y; // Time diff 1 in Y in seconds
	float dt1Z1; // Time diff 1 in Z1 in seconds
	float dt1Z2; // Time diff 1 in Z2 in seconds
	float dt2X; // Time diff 2 in X in seconds
	float dt2Y; // Time diff 2 in Y in seconds
	float dt2Z1; // Time diff 2 in Z1 in seconds
	float dt2Z2; // Time diff 2 in Z2 in seconds
	float timeVelEstX; // Duration of velocity estimate for each component in seconds
	float timeVelEstY; // Duration of velocity estimate for each component in seconds
	float timeVelEstZ1; // Duration of velocity estimate for each component in seconds
	float timeVelEstZ2; // Duration of velocity estimate for each component in seconds
};

#endif // __HGuideAPI_Msg_6721_h__
