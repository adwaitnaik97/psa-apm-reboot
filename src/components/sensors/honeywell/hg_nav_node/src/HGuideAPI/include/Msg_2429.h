#ifndef __HGuideAPI_Msg_2429_h__
#define __HGuideAPI_Msg_2429_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2429 : Transfer Alignment Normalized Residuals
//
// Residuals of generic Transfer align measurements.
// This message reports residuals of experimental body velocity aiding via 0x4110 input
// 
class HGUIDE_DLL Msg_2429
{
public:
	Msg_2429();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 96;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2429; // Message ID
	static const uint32_t MessageLength = 24; // Message Length
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float TA_PositionX_MeasResid; // [sigma]
	float TA_PositionY_MeasResid; // [sigma]
	float TA_PositionZ_MeasResid; // [sigma]
	float TA_VelocityX_MeasResid; // [sigma]
	float TA_VelocityY_MeasResid; // [sigma]
	float TA_VelocityZ_MeasResid; // [sigma]
	float TA_AttitudeX_MeasResid; // [sigma]
	float TA_AttitudeY_MeasResid; // [sigma]
	float TA_AttitudeZ_MeasResid; // [sigma]
	float TA_AttitudeCosZ_MeasResid; // [sigma]
};

#endif // __HGuideAPI_Msg_2429_h__
