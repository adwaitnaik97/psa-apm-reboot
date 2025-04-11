#ifndef __HGuideAPI_Msg_01_h__
#define __HGuideAPI_Msg_01_h__
#pragma once

#include <cstdint>

#include <include/hg4930_status_word_1_t.h>
#include <include/hg4930_multiplex_status_word2_t.h>


// 0x01 : Control Message
//
// It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
// 5/6 frames are in the 0x01, the 6th frame is in the 0x02
// 
class HGUIDE_DLL Msg_01
{
public:
	Msg_01();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 20;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0x01; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	hg4930_status_word_1_t StatusWord1; // Status Word 1
	hg4930_multiplex_status_word2_t MultiPlexedStatusWord2; // Multiplexed Status Word 2
	uint16_t Checksum; // uint16 checksum
};

#endif // __HGuideAPI_Msg_01_h__
