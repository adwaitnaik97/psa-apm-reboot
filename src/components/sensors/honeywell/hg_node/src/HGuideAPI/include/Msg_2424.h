#ifndef __HGuideAPI_Msg_2424_h__
#define __HGuideAPI_Msg_2424_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2424 : GPS Error Estimations
//
// Error estimations for the GNSS system
// 
class HGUIDE_DLL Msg_2424
{
public:
	Msg_2424();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 168;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2424; // Message ID
	static const uint32_t MessageLength = 42; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float gps_clk_phase_error; // [m] GPS Clock Phase Error Estimate
	float gps_clk_freq_error; // [m/s/s] GPS Clock Acceleration Error Estimate
	float gps_clk_acc_error; // [m/s] GPS Clock Frequency Error Estimate
	float gps_clk_g_sensitivity_x; // [ppb/g] GPS Clock Frequency g-sensitivity X Error Estimate
	float gps_clk_g_sensitivity_y; // [ppb/g] GPS Clock Frequency g-sensitivity Y Error Estimate
	float gps_clk_g_sensitivity_z; // [ppb/g] GPS Clock Frequency g-sensitivity Z Error Estimate
	float min_meas_range_bias; // [m] Minimum Successful Measurement Range Bias Error Estimate
	float max_meas_range_bias; // [m] Maximum Successful Measurement Range Bias Error Estimate
	float gps_los_lever_arm_x; // [m] GPS LOS Leverarm X Error Estimate
	float gps_los_lever_arm_y; // [m] GPS LOS Leverarm Y Error Estimate
	float gps_los_lever_arm_z; // [m] GPS LOS Leverarm Z Error Estimate
	float gps_pvt_lever_arm_x; // [m] GPS PVT Leverarm X Error Estimate
	float gps_pvt_lever_arm_y; // [m] GPS PVT Leverarm Y Error Estimate
	float gps_pvt_lever_arm_z; // [m] GPS PVT Leverarm Z Error Estimate
	float gps_position_state_x; // [m] GPS PVT Position X State Estimate
	float gps_position_state_y; // [m] GPS PVT Position Y State Estimate
	float gps_position_state_z; // [m] GPS PVT Position Z State Estimate
	float gps_clk_g_sensitivity_x_stdv; // [ppb/g] GPS Clock Frequency g-sensitivity X STDV
	float gps_clk_g_sensitivity_y_stdv; // [ppb/g] GPS Clock Frequency g-sensitivity Y STDV
	float gps_clk_g_sensitivity_z_stdv; // [ppb/g] GPS Clock Frequency g-sensitivity Z STDV
	float min_meas_range_bias_stdv; // [m] Minimum Successful Measurement Range Bias STDV
	float max_meas_range_bias_stdv; // [m] Maximum Successful Measurement Range Bias STDV
	float gps_los_lever_arm_x_stdv; // [m] GPS LOS Leverarm X STDV
	float gps_los_lever_arm_y_stdv; // [m] GPS LOS Leverarm Y STDV
	float gps_los_lever_arm_z_stdv; // [m] GPS LOS Leverarm Z STDV
	float gps_pvt_lever_arm_x_stdv; // [m] GPS PVT Leverarm X STDV
	float gps_pvt_lever_arm_y_stdv; // [m] GPS PVT Leverarm Y STDV
	float gps_pvt_lever_arm_z_stdv; // [m] GPS PVT Leverarm Z STDV
	float gps_position_state_stdv_x; // [m] GPS PVT Position X State Estimate STDV
	float gps_position_state_stdv_y; // [m] GPS PVT Position Y State Estimate STDV
	float gps_position_state_stdv_z; // [m] GPS PVT Position Z State Estimate STDV
};

#endif // __HGuideAPI_Msg_2424_h__
