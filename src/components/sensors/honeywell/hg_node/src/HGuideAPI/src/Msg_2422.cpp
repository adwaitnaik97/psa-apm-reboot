#include <include/HGuideAPI.h>
#include <include/Msg_2422.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2422::AddressId;
const uint32_t Msg_2422::MessageId;
const uint32_t Msg_2422::MessageLength;

Msg_2422::Msg_2422()
{
	Default();
}

void Msg_2422::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	ecef_pos_x_std_dev = 0;
	ecef_pos_y_std_dev = 0;
	ecef_pos_z_std_dev = 0;
	ecef_vel_x_std_dev = 0;
	ecef_vel_y_std_dev = 0;
	ecef_vel_z_std_dev = 0;
	attitude_roll_std_dev = 0;
	attitude_pitch_std_dev = 0;
	attitude_true_heading_std_dev = 0;
	vehicle_body_x_axis_rotational_angle_std_dev = 0;
	vehicle_body_y_axis_rotational_angle_std_dev = 0;
	vehicle_body_z_axis_rotational_angle_std_dev = 0;
	vehicle_body_x_axis_linear_acceleration_std_dev = 0;
	vehicle_body_y_axis_linear_acceleration_std_dev = 0;
	vehicle_body_z_axis_linear_acceleration_std_dev = 0;
	zero_velocity_x_norm_meas_resid = 0;
	zero_velocity_y_norm_meas_resid = 0;
	zero_velocity_z_norm_meas_resid = 0;
	zero_heading_change_norm_meas_resid = 0;
}

bool Msg_2422::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 140) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INSMode));
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(ecef_pos_x_std_dev);
	bb.setOffset(48);	bb.put(ecef_pos_y_std_dev);
	bb.setOffset(52);	bb.put(ecef_pos_z_std_dev);
	bb.setOffset(56);	bb.put(ecef_vel_x_std_dev);
	bb.setOffset(60);	bb.put(ecef_vel_y_std_dev);
	bb.setOffset(64);	bb.put(ecef_vel_z_std_dev);
	bb.setOffset(68);	bb.put(attitude_roll_std_dev);
	bb.setOffset(72);	bb.put(attitude_pitch_std_dev);
	bb.setOffset(76);	bb.put(attitude_true_heading_std_dev);
	bb.setOffset(80);	bb.put(vehicle_body_x_axis_rotational_angle_std_dev);
	bb.setOffset(84);	bb.put(vehicle_body_y_axis_rotational_angle_std_dev);
	bb.setOffset(88);	bb.put(vehicle_body_z_axis_rotational_angle_std_dev);
	bb.setOffset(92);	bb.put(vehicle_body_x_axis_linear_acceleration_std_dev);
	bb.setOffset(96);	bb.put(vehicle_body_y_axis_linear_acceleration_std_dev);
	bb.setOffset(100);	bb.put(vehicle_body_z_axis_linear_acceleration_std_dev);
	bb.setOffset(104);	bb.put(zero_velocity_x_norm_meas_resid);
	bb.setOffset(108);	bb.put(zero_velocity_y_norm_meas_resid);
	bb.setOffset(112);	bb.put(zero_velocity_z_norm_meas_resid);
	bb.setOffset(124);	bb.put(zero_heading_change_norm_meas_resid);
	Checksum = computeChecksum((uint32_t*)buffer, 35);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2422::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 140) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint32_t AddressId_In;
	bb.setOffset(0);	bb.get(AddressId_In);
	constCheck |= (AddressId != AddressId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageId_In;
	bb.setOffset(4);	bb.get(MessageId_In);
	constCheck |= (MessageId != MessageId_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint32_t MessageLength_In;
	bb.setOffset(8);	bb.get(MessageLength_In);
	constCheck |= (MessageLength != MessageLength_In) << numConsts++;
	bb.setOffset(20);	INSMode = static_cast<ins_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(ecef_pos_x_std_dev);
	bb.setOffset(48);	bb.get(ecef_pos_y_std_dev);
	bb.setOffset(52);	bb.get(ecef_pos_z_std_dev);
	bb.setOffset(56);	bb.get(ecef_vel_x_std_dev);
	bb.setOffset(60);	bb.get(ecef_vel_y_std_dev);
	bb.setOffset(64);	bb.get(ecef_vel_z_std_dev);
	bb.setOffset(68);	bb.get(attitude_roll_std_dev);
	bb.setOffset(72);	bb.get(attitude_pitch_std_dev);
	bb.setOffset(76);	bb.get(attitude_true_heading_std_dev);
	bb.setOffset(80);	bb.get(vehicle_body_x_axis_rotational_angle_std_dev);
	bb.setOffset(84);	bb.get(vehicle_body_y_axis_rotational_angle_std_dev);
	bb.setOffset(88);	bb.get(vehicle_body_z_axis_rotational_angle_std_dev);
	bb.setOffset(92);	bb.get(vehicle_body_x_axis_linear_acceleration_std_dev);
	bb.setOffset(96);	bb.get(vehicle_body_y_axis_linear_acceleration_std_dev);
	bb.setOffset(100);	bb.get(vehicle_body_z_axis_linear_acceleration_std_dev);
	bb.setOffset(104);	bb.get(zero_velocity_x_norm_meas_resid);
	bb.setOffset(108);	bb.get(zero_velocity_y_norm_meas_resid);
	bb.setOffset(112);	bb.get(zero_velocity_z_norm_meas_resid);
	bb.setOffset(124);	bb.get(zero_heading_change_norm_meas_resid);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 35);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

