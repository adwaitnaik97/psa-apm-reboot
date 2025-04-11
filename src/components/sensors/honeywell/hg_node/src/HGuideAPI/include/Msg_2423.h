#ifndef __HGuideAPI_Msg_2423_h__
#define __HGuideAPI_Msg_2423_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2423 : IMU Standard deviation Values
//
// Inertial system standard deviation values
// see 0x2411 for error values
// 
class HGUIDE_DLL Msg_2423
{
public:
	Msg_2423();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 200;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2423; // Message ID
	static const uint32_t MessageLength = 50; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float gyro_bias_x_std_dev; // [deg/h]
	float gyro_bias_y_std_dev; // [deg/h]
	float gyro_bias_z_std_dev; // [deg/h]
	float gyro_bias_inrun_x_std_dev; // [deg/h]
	float gyro_bias_inrun_y_std_dev; // [deg/h]
	float gyro_bias_inrun_z_std_dev; // [deg/h]
	float gyro_scale_factor_x_std_dev; // [ppm]
	float gyro_scale_factor_y_std_dev; // [ppm]
	float gyro_scale_factor_z_std_dev; // [ppm]
	float gyro_nonorthogonality_yz_std_dev; // [urad]
	float gyro_nonorthogonality_zx_std_dev; // [urad]
	float gyro_nonorthogonality_xy_std_dev; // [urad]
	float accelerometer_bias_x_std_dev; // [ug]
	float accelerometer_bias_y_std_dev; // [ug]
	float accelerometer_bias_z_std_dev; // [ug]
	float accelerometer_bias_inrun_x_std_dev; // [ug]
	float accelerometer_bias_inrun_y_std_dev; // [ug]
	float accelerometer_bias_inrun_z_std_dev; // [ug]
	float accelerometer_scale_factor_x_std_dev; // [ppm]
	float accelerometer_scale_factor_y_std_dev; // [ppm]
	float accelerometer_scale_factor_z_std_dev; // [ppm]
	float accelerometer_nonorthogonality_yz_std_dev; // [urad]
	float accelerometer_nonorthogonality_zx_std_dev; // [urad]
	float accelerometer_nonorthogonality_xy_std_dev; // [urad]
	float accelerometer_misalignment_x_std_dev; // [urad]
	float accelerometer_misalignment_y_std_dev; // [urad]
	float accelerometer_misalignment_z_std_dev; // [urad]
	float accelerometer_scale_factor_nonlinearity_x_std_dev; // [ppm]
	float accelerometer_scale_factor_nonlinearity_y_std_dev; // [ppm]
	float accelerometer_scale_factor_nonlinearity_z_std_dev; // [ppm]
};

#endif // __HGuideAPI_Msg_2423_h__
