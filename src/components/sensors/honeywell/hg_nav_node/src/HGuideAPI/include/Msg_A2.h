#ifndef __HGuideAPI_Msg_A2_h__
#define __HGuideAPI_Msg_A2_h__
#pragma once

#include <cstdint>

#include <include/hgimu_status_word_1_t.h>
#include <include/hgimu_multiplex_status_word2_t.h>


// 0xA2 : Navigation Message
//
// It contains the Delta Angle in Rad and Delta Velocity in m/s as well as the 1/6 of Control Data frames
// 
class HGUIDE_DLL Msg_A2
{
public:
	Msg_A2();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 44;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0xA2; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	hgimu_status_word_1_t StatusWord1; // Status Word 1
	hgimu_multiplex_status_word2_t MultiPlexedStatusWord2; // Multiplexed Status Word 2
	double DeltaAngleX; // [rad] Case Frame Delta Angle X
	double DeltaAngleY; // [rad] Case Frame Delta Angle Y
	double DeltaAngleZ; // [rad] Case Frame Delta Angle Z
	double DeltaVelocityX; // [m/sec] Case Frame Delta Velocity X
	double DeltaVelocityY; // [m/sec] Case Frame Delta Velocity Y
	double DeltaVelocityZ; // [m/sec] Case Frame Delta Velocity Z
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_A2_h__
