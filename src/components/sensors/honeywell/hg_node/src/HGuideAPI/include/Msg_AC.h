#ifndef __HGuideAPI_Msg_AC_h__
#define __HGuideAPI_Msg_AC_h__
#pragma once

#include <cstdint>

#include <include/hgimu_status_word_1_t.h>
#include <include/hgimu_multiplex_status_word2_t.h>


// 0xAC : Control Message with Magnetometer data
//
// It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
// 5/6 frames are in the 0xAC, the 6th frame is in the 0xAD
// 
class HGUIDE_DLL Msg_AC
{
public:
	Msg_AC();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 26;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0xAC; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	float MagneticFieldX; // [mGauss] Case Frame Magnetic Flux X
	float MagneticFieldY; // [mGauss] Case Frame Magnetic Flux Y
	float MagneticFieldZ; // [mGauss] Case Frame Magnetic Flux Z
	hgimu_status_word_1_t StatusWord1; // Status Word 1
	hgimu_multiplex_status_word2_t MultiPlexedStatusWord2; // Multiplexed Status Word 2
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_AC_h__
