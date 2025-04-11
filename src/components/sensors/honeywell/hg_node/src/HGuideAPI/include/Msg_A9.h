#ifndef __HGuideAPI_Msg_A9_h__
#define __HGuideAPI_Msg_A9_h__
#pragma once

#include <cstdint>


// 0xA9 : Navigation Message with magnetometer flux
//
// It contains the Delta Angle in Rad and Delta Velocity in m/s and Magnetic Flux in mGauss however no Control Data
// 
class HGUIDE_DLL Msg_A9
{
public:
	Msg_A9();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 56;}

public:
	static const uint8_t SyncByte = 0x0E; // Sync Byte
	static const uint8_t MessageID = 0xA9; // Message ID
	float DeltaAngleX; // [rad] Case Frame Delta Angle X
	float DeltaAngleY; // [rad] Case Frame Delta Angle Y
	float DeltaAngleZ; // [rad] Case Frame Delta Angle Z
	float DeltaVelocityX; // [m/sec] Case Frame Delta Velocity X
	float DeltaVelocityY; // [m/sec] Case Frame Delta Velocity Y
	float DeltaVelocityZ; // [m/sec] Case Frame Delta Velocity Z
	float mg1FluxA; // [mGauss] Case Frame Magnetic Flux X of Mangnetomer #1
	float mg1FluxB; // [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #1
	float mg1FluxC; // [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #1
	float mg1Temperature; // [deg C] Temperature of Mangnetomer #1
	float mg2FluxA; // [mGauss] Case Frame Magnetic Flux X of Mangnetomer #2
	float mg2FluxB; // [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #2
	float mg2FluxC; // [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #2
	float mg2Temperature; // [deg C] Temperature of Mangnetomer #2
	float mg3FluxA; // [mGauss] Case Frame Magnetic Flux X of Mangnetomer #3
	float mg3FluxB; // [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #3
	float mg3FluxC; // [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #3
	float mg3Temperature; // [deg C] Temperature of Mangnetomer #3
	uint16_t Checksum; // 16 bit checksum
};

#endif // __HGuideAPI_Msg_A9_h__
