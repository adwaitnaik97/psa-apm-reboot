#ifndef __HGuideAPI_Msg_2426_h__
#define __HGuideAPI_Msg_2426_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2426 : Barometer errors and uncertainties
//
// Error estimations for the Barometer system
// 
class HGUIDE_DLL Msg_2426
{
public:
	Msg_2426();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 84;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2426; // Message ID
	static const uint32_t MessageLength = 21; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float BaroBiasErrEst;
	float BaroScaleFactorErrEst; // [ppm]
	float BaroBiasSTDV;
	float BaroScaleFactorSTDV; // [ppm]
	float BaroMeasResid;
};

#endif // __HGuideAPI_Msg_2426_h__
