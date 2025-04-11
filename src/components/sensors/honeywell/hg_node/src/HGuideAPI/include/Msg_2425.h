#ifndef __HGuideAPI_Msg_2425_h__
#define __HGuideAPI_Msg_2425_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2425 : Transfer Align Measurement errors and uncertainties
//
// Error estimations for the Transfer Align system
// 
class HGUIDE_DLL Msg_2425
{
public:
	Msg_2425();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 164;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2425; // Message ID
	static const uint32_t MessageLength = 41; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float TA_PosAidingX_ErrEst; // [m]
	float TA_PosAidingY_ErrEst; // [m]
	float TA_PosAidingZ_ErrEst; // [m]
	float TA_VelAidingX_ErrEst; // [m/s]
	float TA_VelAidingY_ErrEst; // [m/s]
	float TA_VelAidingZ_ErrEst; // [m/s]
	float TA_LeverArmX_ErrEst; // [m]
	float TA_LeverArmY_ErrEst; // [m]
	float TA_LeverArmZ_ErrEst; // [m]
	float TA_BoresightX_ErrEst; // [mrad]
	float TA_BoresightY_ErrEst; // [mrad]
	float TA_BoresightZ_ErrEst; // [mrad]
	float TA_PosAidingX_Stdv; // [m]
	float TA_PosAidingY_Stdv; // [m]
	float TA_PosAidingZ_Stdv; // [m]
	float TA_VelAidingX_Stdv; // [m/s]
	float TA_VelAidingY_Stdv; // [m/s]
	float TA_VelAidingZ_Stdv; // [m/s]
	float TA_LeverArmX_Stdv; // [m]
	float TA_LeverArmY_Stdv; // [m]
	float TA_LeverArmZ_Stdv; // [m]
	float TA_BoresightX_Stdv; // [mrad]
	float TA_BoresightY_Stdv; // [mrad]
	float TA_BoresightZ_Stdv; // [mrad]
};

#endif // __HGuideAPI_Msg_2425_h__
