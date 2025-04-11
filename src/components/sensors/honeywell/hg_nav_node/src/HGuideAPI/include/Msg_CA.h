#ifndef __HGuideAPI_Msg_CA_h__
#define __HGuideAPI_Msg_CA_h__
#pragma once

#include <cstdint>

#include <include/hg4930_status_word_1_t.h>
#include <include/hg4930_multiplex_status_word2_t.h>


// 0xCA : Control Message
//
// Used by Platform Stabilization version of HG4930
// It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
// 
class HGUIDE_DLL Msg_CA
{
public:
	Msg_CA();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 20;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0xCA; // Message ID
	float AngularRateX; // [rad/sec] Case Frame Angular Rate X
	float AngularRateY; // [rad/sec] Case Frame Angular Rate Y
	float AngularRateZ; // [rad/sec] Case Frame Angular Rate Z
	float LinearAccelerationX; // [m/sec/sec] Case Frame Linear Acceleration X
	float LinearAccelerationY; // [m/sec/sec] Case Frame Linear Acceleration Y
	float LinearAccelerationZ; // [m/sec/sec] Case Frame Linear Acceleration Z
	hg4930_status_word_1_t StatusWord1; // Status Word 1
	hg4930_multiplex_status_word2_t MultiPlexedStatusWord2; // Multiplexed Status Word 2
	uint16_t Checksum; // uint16 checksum. No complements or anything, just adds all of the data as if it were a uint16.
};

#endif // __HGuideAPI_Msg_CA_h__
