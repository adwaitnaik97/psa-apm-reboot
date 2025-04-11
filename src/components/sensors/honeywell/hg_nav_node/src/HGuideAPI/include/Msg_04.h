#ifndef __HGuideAPI_Msg_04_h__
#define __HGuideAPI_Msg_04_h__
#pragma once

#include <cstdint>

#include <include/hg1120_status_word_1_t.h>
#include <include/hg1120_multiplex_status_word2_t.h>


// 0x04 : Control Message
//
// It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
// 5/6 frames are in the 0x04, the 6th frame is in the 0x05
// 
class HGUIDE_DLL Msg_04
{
public:
	Msg_04();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 26;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0x04; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	float MagneticFieldX; // [mGauss] Case Frame Linear Acceleration X
	float MagneticFieldY; // [mGauss] Case Frame Linear Acceleration Y
	float MagneticFieldZ; // [mGauss] Case Frame Linear Acceleration Z
	hg1120_status_word_1_t MainStatusWord; // Main Status Word
	hg1120_multiplex_status_word2_t MultiPlexedStatusWord; // Multiplexed Status Word
	uint16_t Checksum; // uint16 checksum. No complements or anything, just adds all of the data as if it were a uint16.
};

#endif // __HGuideAPI_Msg_04_h__
