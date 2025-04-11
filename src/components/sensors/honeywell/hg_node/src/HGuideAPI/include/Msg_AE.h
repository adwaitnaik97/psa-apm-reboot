#ifndef __HGuideAPI_Msg_AE_h__
#define __HGuideAPI_Msg_AE_h__
#pragma once

#include <cstdint>

#include <include/hgimu_status_word_1_t.h>
#include <include/hgimu_multiplex_status_word2_t.h>


// 0xAE : Navigation Message with Magnetometer data and without Control Data
//
// It contains the Delta Angle in Rad and Delta Velocity in m/s and magnetic field in mGauss
// 
class HGUIDE_DLL Msg_AE
{
public:
	Msg_AE();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 38;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0xAE; // Message ID
	double DeltaAngleX; // [rad] Case Frame Delta Angle X
	double DeltaAngleY; // [rad] Case Frame Delta Angle Y
	double DeltaAngleZ; // [rad] Case Frame Delta Angle Z
	double DeltaVelocityX; // [m/sec] Case Frame Delta Velocity X
	double DeltaVelocityY; // [m/sec] Case Frame Delta Velocity Y
	double DeltaVelocityZ; // [m/sec] Case Frame Delta Velocity Z
	float MagneticFieldX; // [mGauss] Case Frame Magnetic Flux X
	float MagneticFieldY; // [mGauss] Case Frame Magnetic Flux Y
	float MagneticFieldZ; // [mGauss] Case Frame Magnetic Flux Z
	hgimu_status_word_1_t StatusWord1; // Status Word 1
	hgimu_multiplex_status_word2_t MultiPlexedStatusWord2; // Multiplexed Status Word 2
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_AE_h__
