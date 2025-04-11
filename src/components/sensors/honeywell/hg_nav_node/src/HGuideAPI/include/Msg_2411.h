#ifndef __HGuideAPI_Msg_2411_h__
#define __HGuideAPI_Msg_2411_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2411 : IMU Error Values
//
// Inertial system error values
// See 0x2423 for standard deviations
// 
class HGUIDE_DLL Msg_2411
{
public:
	Msg_2411();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 188;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2411; // Message ID
	static const uint32_t MessageLength = 47; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float gyro_bias_error_x; // [deg/h]
	float gyro_bias_error_y; // [deg/h]
	float gyro_bias_error_z; // [deg/h]
	float gyro_bias_inrun_error_x; // [deg/h]
	float gyro_bias_inrun_error_y; // [deg/h]
	float gyro_bias_inrun_error_z; // [deg/h]
	float gyro_scale_factor_error_x; // [ppm]
	float gyro_scale_factor_error_y; // [ppm]
	float gyro_scale_factor_error_z; // [ppm]
	float gyro_nonorthogonality_error_yz; // [urad]
	float gyro_nonorthogonality_error_zx; // [urad]
	float gyro_nonorthogonality_error_xy; // [urad]
	float accelerometer_bias_error_x; // [ug]
	float accelerometer_bias_error_y; // [ug]
	float accelerometer_bias_error_z; // [ug]
	float accelerometer_bias_inrun_error_x; // [ug]
	float accelerometer_bias_inrun_error_y; // [ug]
	float accelerometer_bias_inrun_error_z; // [ug]
	float accelerometer_scale_factor_error_x; // [ppm]
	float accelerometer_scale_factor_error_y; // [ppm]
	float accelerometer_scale_factor_error_z; // [ppm]
	float accelerometer_nonorthogonality_error_yz; // [urad]
	float accelerometer_nonorthogonality_error_zx; // [urad]
	float accelerometer_nonorthogonality_error_xy; // [urad]
	float accelerometer_misalignment_error_x; // [urad]
	float accelerometer_misalignment_error_y; // [urad]
	float accelerometer_misalignment_error_z; // [urad]
	float accelerometer_scale_factor_nonlinearity_error_x; // [ppm]
	float accelerometer_scale_factor_nonlinearity_error_y; // [ppm]
	float accelerometer_scale_factor_nonlinearity_error_z; // [ppm]
};

#endif // __HGuideAPI_Msg_2411_h__
