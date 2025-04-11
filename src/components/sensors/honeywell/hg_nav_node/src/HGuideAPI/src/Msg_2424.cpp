#include <include/HGuideAPI.h>
#include <include/Msg_2424.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2424::AddressId;
const uint32_t Msg_2424::MessageId;
const uint32_t Msg_2424::MessageLength;

Msg_2424::Msg_2424()
{
	Default();
}

void Msg_2424::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	gps_clk_phase_error = 0;
	gps_clk_freq_error = 0;
	gps_clk_acc_error = 0;
	gps_clk_g_sensitivity_x = 0;
	gps_clk_g_sensitivity_y = 0;
	gps_clk_g_sensitivity_z = 0;
	min_meas_range_bias = 0;
	max_meas_range_bias = 0;
	gps_los_lever_arm_x = 0;
	gps_los_lever_arm_y = 0;
	gps_los_lever_arm_z = 0;
	gps_pvt_lever_arm_x = 0;
	gps_pvt_lever_arm_y = 0;
	gps_pvt_lever_arm_z = 0;
	gps_position_state_x = 0;
	gps_position_state_y = 0;
	gps_position_state_z = 0;
	gps_clk_g_sensitivity_x_stdv = 0;
	gps_clk_g_sensitivity_y_stdv = 0;
	gps_clk_g_sensitivity_z_stdv = 0;
	min_meas_range_bias_stdv = 0;
	max_meas_range_bias_stdv = 0;
	gps_los_lever_arm_x_stdv = 0;
	gps_los_lever_arm_y_stdv = 0;
	gps_los_lever_arm_z_stdv = 0;
	gps_pvt_lever_arm_x_stdv = 0;
	gps_pvt_lever_arm_y_stdv = 0;
	gps_pvt_lever_arm_z_stdv = 0;
	gps_position_state_stdv_x = 0;
	gps_position_state_stdv_y = 0;
	gps_position_state_stdv_z = 0;
}

bool Msg_2424::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 168) return false;

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
	bb.setOffset(44);	bb.put(gps_clk_phase_error);
	bb.setOffset(48);	bb.put(gps_clk_freq_error);
	bb.setOffset(52);	bb.put(gps_clk_acc_error);
	bb.setOffset(56);	bb.put(gps_clk_g_sensitivity_x);
	bb.setOffset(60);	bb.put(gps_clk_g_sensitivity_y);
	bb.setOffset(64);	bb.put(gps_clk_g_sensitivity_z);
	bb.setOffset(68);	bb.put(min_meas_range_bias);
	bb.setOffset(72);	bb.put(max_meas_range_bias);
	bb.setOffset(76);	bb.put(gps_los_lever_arm_x);
	bb.setOffset(80);	bb.put(gps_los_lever_arm_y);
	bb.setOffset(84);	bb.put(gps_los_lever_arm_z);
	bb.setOffset(88);	bb.put(gps_pvt_lever_arm_x);
	bb.setOffset(92);	bb.put(gps_pvt_lever_arm_y);
	bb.setOffset(96);	bb.put(gps_pvt_lever_arm_z);
	bb.setOffset(100);	bb.put(gps_position_state_x);
	bb.setOffset(104);	bb.put(gps_position_state_y);
	bb.setOffset(108);	bb.put(gps_position_state_z);
	bb.setOffset(112);	bb.put(gps_clk_g_sensitivity_x_stdv);
	bb.setOffset(116);	bb.put(gps_clk_g_sensitivity_y_stdv);
	bb.setOffset(120);	bb.put(gps_clk_g_sensitivity_z_stdv);
	bb.setOffset(124);	bb.put(min_meas_range_bias_stdv);
	bb.setOffset(128);	bb.put(max_meas_range_bias_stdv);
	bb.setOffset(132);	bb.put(gps_los_lever_arm_x_stdv);
	bb.setOffset(136);	bb.put(gps_los_lever_arm_y_stdv);
	bb.setOffset(140);	bb.put(gps_los_lever_arm_z_stdv);
	bb.setOffset(144);	bb.put(gps_pvt_lever_arm_x_stdv);
	bb.setOffset(148);	bb.put(gps_pvt_lever_arm_y_stdv);
	bb.setOffset(152);	bb.put(gps_pvt_lever_arm_z_stdv);
	bb.setOffset(156);	bb.put(gps_position_state_stdv_x);
	bb.setOffset(160);	bb.put(gps_position_state_stdv_y);
	bb.setOffset(164);	bb.put(gps_position_state_stdv_z);
	Checksum = computeChecksum((uint32_t*)buffer, 42);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2424::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 168) return -2;

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
	bb.setOffset(44);	bb.get(gps_clk_phase_error);
	bb.setOffset(48);	bb.get(gps_clk_freq_error);
	bb.setOffset(52);	bb.get(gps_clk_acc_error);
	bb.setOffset(56);	bb.get(gps_clk_g_sensitivity_x);
	bb.setOffset(60);	bb.get(gps_clk_g_sensitivity_y);
	bb.setOffset(64);	bb.get(gps_clk_g_sensitivity_z);
	bb.setOffset(68);	bb.get(min_meas_range_bias);
	bb.setOffset(72);	bb.get(max_meas_range_bias);
	bb.setOffset(76);	bb.get(gps_los_lever_arm_x);
	bb.setOffset(80);	bb.get(gps_los_lever_arm_y);
	bb.setOffset(84);	bb.get(gps_los_lever_arm_z);
	bb.setOffset(88);	bb.get(gps_pvt_lever_arm_x);
	bb.setOffset(92);	bb.get(gps_pvt_lever_arm_y);
	bb.setOffset(96);	bb.get(gps_pvt_lever_arm_z);
	bb.setOffset(100);	bb.get(gps_position_state_x);
	bb.setOffset(104);	bb.get(gps_position_state_y);
	bb.setOffset(108);	bb.get(gps_position_state_z);
	bb.setOffset(112);	bb.get(gps_clk_g_sensitivity_x_stdv);
	bb.setOffset(116);	bb.get(gps_clk_g_sensitivity_y_stdv);
	bb.setOffset(120);	bb.get(gps_clk_g_sensitivity_z_stdv);
	bb.setOffset(124);	bb.get(min_meas_range_bias_stdv);
	bb.setOffset(128);	bb.get(max_meas_range_bias_stdv);
	bb.setOffset(132);	bb.get(gps_los_lever_arm_x_stdv);
	bb.setOffset(136);	bb.get(gps_los_lever_arm_y_stdv);
	bb.setOffset(140);	bb.get(gps_los_lever_arm_z_stdv);
	bb.setOffset(144);	bb.get(gps_pvt_lever_arm_x_stdv);
	bb.setOffset(148);	bb.get(gps_pvt_lever_arm_y_stdv);
	bb.setOffset(152);	bb.get(gps_pvt_lever_arm_z_stdv);
	bb.setOffset(156);	bb.get(gps_position_state_stdv_x);
	bb.setOffset(160);	bb.get(gps_position_state_stdv_y);
	bb.setOffset(164);	bb.get(gps_position_state_stdv_z);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 42);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

