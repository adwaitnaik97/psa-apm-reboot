#ifndef __HGuideAPI_Msg_0C_h__
#define __HGuideAPI_Msg_0C_h__
#pragma once

#include <cstdint>

#include <include/hg1120_status_word_1_t.h>
#include <include/hg1120_multiplex_status_word2_t.h>


// 0x0C : Control Message
//
// It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
// 5/6 frames are in the 0x0C, the 6th frame is in the 0x0D
// 
class HGUIDE_DLL Msg_0C
{
public:
	Msg_0C();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 26;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0x0C; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	float MagneticFieldX; // [mGauss] Case Frame Magnetic Flux X
	float MagneticFieldY; // [mGauss] Case Frame Magnetic Flux Y
	float MagneticFieldZ; // [mGauss] Case Frame Magnetic Flux Z
	hg1120_status_word_1_t MainStatusWord; // Main Status Word
	hg1120_multiplex_status_word2_t MultiPlexedStatusWord; // Multiplexed Status Word
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_0C_h__
