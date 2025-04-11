#ifndef __HGuideAPI_Msg_2422_h__
#define __HGuideAPI_Msg_2422_h__
#pragma once

#include <cstdint>

#include <include/ins_mode_table_t.h>


// 0x2422 : Filter Navigation Error Values
//
// Filter error values and standard deviations of Navigation solution
// 
class HGUIDE_DLL Msg_2422
{
public:
	Msg_2422();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 140;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2422; // Message ID
	static const uint32_t MessageLength = 35; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	ins_mode_table_t INSMode; // INS Mode table
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float ecef_pos_x_std_dev; // [m]
	float ecef_pos_y_std_dev; // [m]
	float ecef_pos_z_std_dev; // [m]
	float ecef_vel_x_std_dev; // [m/s]
	float ecef_vel_y_std_dev; // [m/s]
	float ecef_vel_z_std_dev; // [m/s]
	float attitude_roll_std_dev; // [rad]
	float attitude_pitch_std_dev; // [rad]
	float attitude_true_heading_std_dev; // [rad]
	float vehicle_body_x_axis_rotational_angle_std_dev;
	float vehicle_body_y_axis_rotational_angle_std_dev;
	float vehicle_body_z_axis_rotational_angle_std_dev;
	uint32_t vehicle_body_x_axis_linear_acceleration_std_dev;
	uint32_t vehicle_body_y_axis_linear_acceleration_std_dev;
	uint32_t vehicle_body_z_axis_linear_acceleration_std_dev;
	float zero_velocity_x_norm_meas_resid;
	float zero_velocity_y_norm_meas_resid;
	float zero_velocity_z_norm_meas_resid;
	float zero_heading_change_norm_meas_resid;
};

#endif // __HGuideAPI_Msg_2422_h__
