#include <include/HGuideAPI.h>
#include <include/Msg_2501.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2501::AddressId;
const uint32_t Msg_2501::MessageId;
const uint32_t Msg_2501::MessageLength;

Msg_2501::Msg_2501()
{
	Default();
}

void Msg_2501::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	ChannelIdentifier = 0;
	channel_status_1.Default();
	channel_status_2.Default();
	channel_status_3.Default();
	channel_status_4.Default();
	channel_status_5.Default();
	channel_status_6.Default();
}

bool Msg_2501::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 280) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(gps_week);
	bb.setOffset(36);	bb.put(ChannelIdentifier);

	uint32_t channel_status_1_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_1_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[0], 0, 7, channel_status_1.satellite_num, status_ok);
	channel_status_1_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[0], 8, 15, channel_status_1.hrdw_channel_num, status_ok);
	channel_status_1_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[0], 16, 23, channel_status_1.channel_state, status_ok);
	channel_status_1_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[0], 24, 30, channel_status_1.channel_code, status_ok);
	channel_status_1_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_1_tmp[0], 31, channel_status_1.used_in_pvt_sln, status_ok);
	channel_status_1_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_1.carrier_to_noise / (0.1)), status_ok);
	channel_status_1_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[1], 16, 23, channel_status_1.channel_fqcy, status_ok);
	channel_status_1_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[2], 0, 7, channel_status_1.delta_range_valid, status_ok);
	channel_status_1_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[2], 8, 15, channel_status_1.pseudo_range_valid, status_ok);
	channel_status_1_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_1.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_1_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_1.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_1_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[4], 0, 7, channel_status_1.iono_comp_data_src, status_ok);
	channel_status_1_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[4], 8, 15, channel_status_1.sat_sel_desel, status_ok);
	channel_status_1_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_1.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_1_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_1.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_1_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_1.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_1_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_1.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_1_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_1_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_1.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(40);	bb.put(channel_status_1_tmp);


	uint32_t channel_status_2_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_2_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[0], 0, 7, channel_status_2.satellite_num, status_ok);
	channel_status_2_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[0], 8, 15, channel_status_2.hrdw_channel_num, status_ok);
	channel_status_2_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[0], 16, 23, channel_status_2.channel_state, status_ok);
	channel_status_2_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[0], 24, 30, channel_status_2.channel_code, status_ok);
	channel_status_2_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_2_tmp[0], 31, channel_status_2.used_in_pvt_sln, status_ok);
	channel_status_2_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_2.carrier_to_noise / (0.1)), status_ok);
	channel_status_2_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[1], 16, 23, channel_status_2.channel_fqcy, status_ok);
	channel_status_2_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[2], 0, 7, channel_status_2.delta_range_valid, status_ok);
	channel_status_2_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[2], 8, 15, channel_status_2.pseudo_range_valid, status_ok);
	channel_status_2_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_2.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_2_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_2.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_2_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[4], 0, 7, channel_status_2.iono_comp_data_src, status_ok);
	channel_status_2_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[4], 8, 15, channel_status_2.sat_sel_desel, status_ok);
	channel_status_2_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_2.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_2_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_2.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_2_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_2.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_2_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_2.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_2_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_2_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_2.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(80);	bb.put(channel_status_2_tmp);


	uint32_t channel_status_3_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_3_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[0], 0, 7, channel_status_3.satellite_num, status_ok);
	channel_status_3_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[0], 8, 15, channel_status_3.hrdw_channel_num, status_ok);
	channel_status_3_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[0], 16, 23, channel_status_3.channel_state, status_ok);
	channel_status_3_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[0], 24, 30, channel_status_3.channel_code, status_ok);
	channel_status_3_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_3_tmp[0], 31, channel_status_3.used_in_pvt_sln, status_ok);
	channel_status_3_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_3.carrier_to_noise / (0.1)), status_ok);
	channel_status_3_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[1], 16, 23, channel_status_3.channel_fqcy, status_ok);
	channel_status_3_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[2], 0, 7, channel_status_3.delta_range_valid, status_ok);
	channel_status_3_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[2], 8, 15, channel_status_3.pseudo_range_valid, status_ok);
	channel_status_3_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_3.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_3_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_3.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_3_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[4], 0, 7, channel_status_3.iono_comp_data_src, status_ok);
	channel_status_3_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[4], 8, 15, channel_status_3.sat_sel_desel, status_ok);
	channel_status_3_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_3.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_3_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_3.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_3_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_3.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_3_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_3.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_3_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_3_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_3.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(120);	bb.put(channel_status_3_tmp);


	uint32_t channel_status_4_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_4_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[0], 0, 7, channel_status_4.satellite_num, status_ok);
	channel_status_4_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[0], 8, 15, channel_status_4.hrdw_channel_num, status_ok);
	channel_status_4_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[0], 16, 23, channel_status_4.channel_state, status_ok);
	channel_status_4_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[0], 24, 30, channel_status_4.channel_code, status_ok);
	channel_status_4_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_4_tmp[0], 31, channel_status_4.used_in_pvt_sln, status_ok);
	channel_status_4_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_4.carrier_to_noise / (0.1)), status_ok);
	channel_status_4_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[1], 16, 23, channel_status_4.channel_fqcy, status_ok);
	channel_status_4_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[2], 0, 7, channel_status_4.delta_range_valid, status_ok);
	channel_status_4_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[2], 8, 15, channel_status_4.pseudo_range_valid, status_ok);
	channel_status_4_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_4.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_4_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_4.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_4_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[4], 0, 7, channel_status_4.iono_comp_data_src, status_ok);
	channel_status_4_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[4], 8, 15, channel_status_4.sat_sel_desel, status_ok);
	channel_status_4_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_4.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_4_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_4.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_4_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_4.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_4_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_4.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_4_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_4_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_4.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(160);	bb.put(channel_status_4_tmp);


	uint32_t channel_status_5_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_5_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[0], 0, 7, channel_status_5.satellite_num, status_ok);
	channel_status_5_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[0], 8, 15, channel_status_5.hrdw_channel_num, status_ok);
	channel_status_5_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[0], 16, 23, channel_status_5.channel_state, status_ok);
	channel_status_5_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[0], 24, 30, channel_status_5.channel_code, status_ok);
	channel_status_5_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_5_tmp[0], 31, channel_status_5.used_in_pvt_sln, status_ok);
	channel_status_5_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_5.carrier_to_noise / (0.1)), status_ok);
	channel_status_5_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[1], 16, 23, channel_status_5.channel_fqcy, status_ok);
	channel_status_5_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[2], 0, 7, channel_status_5.delta_range_valid, status_ok);
	channel_status_5_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[2], 8, 15, channel_status_5.pseudo_range_valid, status_ok);
	channel_status_5_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_5.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_5_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_5.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_5_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[4], 0, 7, channel_status_5.iono_comp_data_src, status_ok);
	channel_status_5_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[4], 8, 15, channel_status_5.sat_sel_desel, status_ok);
	channel_status_5_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_5.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_5_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_5.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_5_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_5.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_5_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_5.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_5_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_5_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_5.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(200);	bb.put(channel_status_5_tmp);


	uint32_t channel_status_6_tmp[10] = { 0 }; // buffer holding the custom bitfield
	channel_status_6_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[0], 0, 7, channel_status_6.satellite_num, status_ok);
	channel_status_6_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[0], 8, 15, channel_status_6.hrdw_channel_num, status_ok);
	channel_status_6_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[0], 16, 23, channel_status_6.channel_state, status_ok);
	channel_status_6_tmp[0] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[0], 24, 30, channel_status_6.channel_code, status_ok);
	channel_status_6_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(channel_status_6_tmp[0], 31, channel_status_6.used_in_pvt_sln, status_ok);
	channel_status_6_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[1], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_6.carrier_to_noise / (0.1)), status_ok);
	channel_status_6_tmp[1] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[1], 16, 23, channel_status_6.channel_fqcy, status_ok);
	channel_status_6_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[2], 0, 7, channel_status_6.delta_range_valid, status_ok);
	channel_status_6_tmp[2] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[2], 8, 15, channel_status_6.pseudo_range_valid, status_ok);
	channel_status_6_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[3], 0, 15, static_cast<int16_t>CHECK_MAX_INT16(channel_status_6.azimuth / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_6_tmp[3] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[3], 16, 31, static_cast<int16_t>CHECK_MAX_INT16(channel_status_6.elevation / (ECTOS::CONSTANTS::pi*std::pow(2, -15))), status_ok);
	channel_status_6_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[4], 0, 7, channel_status_6.iono_comp_data_src, status_ok);
	channel_status_6_tmp[4] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[4], 8, 15, channel_status_6.sat_sel_desel, status_ok);
	channel_status_6_tmp[5] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[5], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_6.pseudo_range_meas / (std::pow(2, -6))), status_ok);
	channel_status_6_tmp[6] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[6], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_6.delta_range_meas / (std::pow(2, -13))), status_ok);
	channel_status_6_tmp[7] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[7], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_6.sv_pos_x / (std::pow(2, -6))), status_ok);
	channel_status_6_tmp[8] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[8], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_6.sv_pos_y / (std::pow(2, -6))), status_ok);
	channel_status_6_tmp[9] = ECTOS::BIT_UTILITIES::Pack(channel_status_6_tmp[9], 0, 31, static_cast<int32_t>CHECK_MAX_INT32(channel_status_6.sv_pos_z / (std::pow(2, -6))), status_ok);
	bb.setOffset(240);	bb.put(channel_status_6_tmp);

	Checksum = computeChecksum((uint32_t*)buffer, 70);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2501::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 280) return -2;

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
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);	bb.get(gpsTov);
	bb.setOffset(32);	bb.get(gps_week);
	bb.setOffset(36);	bb.get(ChannelIdentifier);

	uint32_t channel_status_1_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(40);	bb.get(channel_status_1_tmp);
	channel_status_1.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[0], 0, 7,  status_ok);
	channel_status_1.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[0], 8, 15,  status_ok);
	channel_status_1.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[0], 16, 23,  status_ok);
	channel_status_1.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[0], 24, 30,  status_ok);
	channel_status_1.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_1_tmp[0], 31,  status_ok);
	channel_status_1.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_1_tmp+1)*((0.1)));
	channel_status_1.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[1], 16, 23,  status_ok);
	channel_status_1.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[2], 0, 7,  status_ok);
	channel_status_1.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[2], 8, 15,  status_ok);
	channel_status_1.azimuth= static_cast<float>(*(int16_t*)(channel_status_1_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_1.elevation= static_cast<float>(*(int16_t*)(channel_status_1_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_1.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[4], 0, 7,  status_ok);
	channel_status_1.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_1_tmp[4], 8, 15,  status_ok);
	channel_status_1.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_1_tmp+5)*((std::pow(2, -6))));
	channel_status_1.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_1_tmp+6)*((std::pow(2, -13))));
	channel_status_1.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_1_tmp+7)*((std::pow(2, -6))));
	channel_status_1.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_1_tmp+8)*((std::pow(2, -6))));
	channel_status_1.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_1_tmp+9)*((std::pow(2, -6))));

	uint32_t channel_status_2_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(80);	bb.get(channel_status_2_tmp);
	channel_status_2.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[0], 0, 7,  status_ok);
	channel_status_2.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[0], 8, 15,  status_ok);
	channel_status_2.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[0], 16, 23,  status_ok);
	channel_status_2.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[0], 24, 30,  status_ok);
	channel_status_2.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_2_tmp[0], 31,  status_ok);
	channel_status_2.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_2_tmp+1)*((0.1)));
	channel_status_2.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[1], 16, 23,  status_ok);
	channel_status_2.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[2], 0, 7,  status_ok);
	channel_status_2.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[2], 8, 15,  status_ok);
	channel_status_2.azimuth= static_cast<float>(*(int16_t*)(channel_status_2_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_2.elevation= static_cast<float>(*(int16_t*)(channel_status_2_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_2.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[4], 0, 7,  status_ok);
	channel_status_2.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_2_tmp[4], 8, 15,  status_ok);
	channel_status_2.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_2_tmp+5)*((std::pow(2, -6))));
	channel_status_2.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_2_tmp+6)*((std::pow(2, -13))));
	channel_status_2.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_2_tmp+7)*((std::pow(2, -6))));
	channel_status_2.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_2_tmp+8)*((std::pow(2, -6))));
	channel_status_2.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_2_tmp+9)*((std::pow(2, -6))));

	uint32_t channel_status_3_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(120);	bb.get(channel_status_3_tmp);
	channel_status_3.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[0], 0, 7,  status_ok);
	channel_status_3.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[0], 8, 15,  status_ok);
	channel_status_3.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[0], 16, 23,  status_ok);
	channel_status_3.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[0], 24, 30,  status_ok);
	channel_status_3.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_3_tmp[0], 31,  status_ok);
	channel_status_3.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_3_tmp+1)*((0.1)));
	channel_status_3.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[1], 16, 23,  status_ok);
	channel_status_3.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[2], 0, 7,  status_ok);
	channel_status_3.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[2], 8, 15,  status_ok);
	channel_status_3.azimuth= static_cast<float>(*(int16_t*)(channel_status_3_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_3.elevation= static_cast<float>(*(int16_t*)(channel_status_3_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_3.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[4], 0, 7,  status_ok);
	channel_status_3.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_3_tmp[4], 8, 15,  status_ok);
	channel_status_3.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_3_tmp+5)*((std::pow(2, -6))));
	channel_status_3.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_3_tmp+6)*((std::pow(2, -13))));
	channel_status_3.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_3_tmp+7)*((std::pow(2, -6))));
	channel_status_3.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_3_tmp+8)*((std::pow(2, -6))));
	channel_status_3.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_3_tmp+9)*((std::pow(2, -6))));

	uint32_t channel_status_4_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(160);	bb.get(channel_status_4_tmp);
	channel_status_4.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[0], 0, 7,  status_ok);
	channel_status_4.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[0], 8, 15,  status_ok);
	channel_status_4.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[0], 16, 23,  status_ok);
	channel_status_4.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[0], 24, 30,  status_ok);
	channel_status_4.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_4_tmp[0], 31,  status_ok);
	channel_status_4.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_4_tmp+1)*((0.1)));
	channel_status_4.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[1], 16, 23,  status_ok);
	channel_status_4.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[2], 0, 7,  status_ok);
	channel_status_4.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[2], 8, 15,  status_ok);
	channel_status_4.azimuth= static_cast<float>(*(int16_t*)(channel_status_4_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_4.elevation= static_cast<float>(*(int16_t*)(channel_status_4_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_4.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[4], 0, 7,  status_ok);
	channel_status_4.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_4_tmp[4], 8, 15,  status_ok);
	channel_status_4.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_4_tmp+5)*((std::pow(2, -6))));
	channel_status_4.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_4_tmp+6)*((std::pow(2, -13))));
	channel_status_4.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_4_tmp+7)*((std::pow(2, -6))));
	channel_status_4.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_4_tmp+8)*((std::pow(2, -6))));
	channel_status_4.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_4_tmp+9)*((std::pow(2, -6))));

	uint32_t channel_status_5_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(200);	bb.get(channel_status_5_tmp);
	channel_status_5.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[0], 0, 7,  status_ok);
	channel_status_5.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[0], 8, 15,  status_ok);
	channel_status_5.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[0], 16, 23,  status_ok);
	channel_status_5.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[0], 24, 30,  status_ok);
	channel_status_5.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_5_tmp[0], 31,  status_ok);
	channel_status_5.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_5_tmp+1)*((0.1)));
	channel_status_5.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[1], 16, 23,  status_ok);
	channel_status_5.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[2], 0, 7,  status_ok);
	channel_status_5.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[2], 8, 15,  status_ok);
	channel_status_5.azimuth= static_cast<float>(*(int16_t*)(channel_status_5_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_5.elevation= static_cast<float>(*(int16_t*)(channel_status_5_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_5.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[4], 0, 7,  status_ok);
	channel_status_5.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_5_tmp[4], 8, 15,  status_ok);
	channel_status_5.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_5_tmp+5)*((std::pow(2, -6))));
	channel_status_5.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_5_tmp+6)*((std::pow(2, -13))));
	channel_status_5.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_5_tmp+7)*((std::pow(2, -6))));
	channel_status_5.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_5_tmp+8)*((std::pow(2, -6))));
	channel_status_5.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_5_tmp+9)*((std::pow(2, -6))));

	uint32_t channel_status_6_tmp[10] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(240);	bb.get(channel_status_6_tmp);
	channel_status_6.satellite_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[0], 0, 7,  status_ok);
	channel_status_6.hrdw_channel_num = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[0], 8, 15,  status_ok);
	channel_status_6.channel_state = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[0], 16, 23,  status_ok);
	channel_status_6.channel_code = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[0], 24, 30,  status_ok);
	channel_status_6.used_in_pvt_sln = ECTOS::BIT_UTILITIES::UnPackBool(channel_status_6_tmp[0], 31,  status_ok);
	channel_status_6.carrier_to_noise= static_cast<float>(*(int16_t*)(channel_status_6_tmp+1)*((0.1)));
	channel_status_6.channel_fqcy = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[1], 16, 23,  status_ok);
	channel_status_6.delta_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[2], 0, 7,  status_ok);
	channel_status_6.pseudo_range_valid = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[2], 8, 15,  status_ok);
	channel_status_6.azimuth= static_cast<float>(*(int16_t*)(channel_status_6_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_6.elevation= static_cast<float>(*(int16_t*)(channel_status_6_tmp+3)*((ECTOS::CONSTANTS::pi*std::pow(2, -15))));
	channel_status_6.iono_comp_data_src = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[4], 0, 7,  status_ok);
	channel_status_6.sat_sel_desel = ECTOS::BIT_UTILITIES::UnPack(channel_status_6_tmp[4], 8, 15,  status_ok);
	channel_status_6.pseudo_range_meas= static_cast<float>(*(int32_t*)(channel_status_6_tmp+5)*((std::pow(2, -6))));
	channel_status_6.delta_range_meas= static_cast<float>(*(int32_t*)(channel_status_6_tmp+6)*((std::pow(2, -13))));
	channel_status_6.sv_pos_x= static_cast<float>(*(int32_t*)(channel_status_6_tmp+7)*((std::pow(2, -6))));
	channel_status_6.sv_pos_y= static_cast<float>(*(int32_t*)(channel_status_6_tmp+8)*((std::pow(2, -6))));
	channel_status_6.sv_pos_z= static_cast<float>(*(int32_t*)(channel_status_6_tmp+9)*((std::pow(2, -6))));
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 70);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

