#include <include/HGuideAPI.h>
#include <include/Msg_1003.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1003::AddressId;
const uint32_t Msg_1003::MessageId;
const uint32_t Msg_1003::MessageLength;

Msg_1003::Msg_1003()
{
	Default();
}

void Msg_1003::Default()
{
	Checksum = 0;
	anti_jam_selection = 0;
	gps_antenna_spin = 0;
	constellation_type = 0;
	acquisition_state_selection = 0;
	ionosphere_corrections = 0;
	post_launch_auto_transition_to_trk = 0;
	set_back_shock = 0;
	position_uncertainty = 0;
	velocity_uncertainty = 0;
	time_uncertainty = 0;
	number_of_acquisition_attempts = 0;
	gps_oscillator_calibration_jitter = 0;
	gps_oscillator_calibration = 0;
	gps_code_select = 0;
	gps_antenna_frequency_cmd = 0;
	gps_svid_deselect_1_4 = 0;
	gps_svid_deselect_1_4_2 = 0;
	pre_launch_temperature_ramp = 0;
	post_launch_temperature_ramp = 0;
	post_launch_power_on_time_delay = 0;
	post_launch_power_on_time_delay_uncertainty = 0;
}

bool Msg_1003::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 176) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(anti_jam_selection);
	bb.setOffset(20);	bb.put(gps_antenna_spin);
	bb.setOffset(24);	bb.put(constellation_type);
	bb.setOffset(28);	bb.put(acquisition_state_selection);
	bb.setOffset(32);	bb.put(ionosphere_corrections);
	bb.setOffset(36);	bb.put(post_launch_auto_transition_to_trk);
	bb.setOffset(40);	bb.put(set_back_shock);
	bb.setOffset(44);	bb.put(position_uncertainty);
	bb.setOffset(48);	bb.put(velocity_uncertainty);
	bb.setOffset(52);	bb.put(time_uncertainty);
	bb.setOffset(60);	bb.put(number_of_acquisition_attempts);
	bb.setOffset(64);	bb.put(gps_oscillator_calibration_jitter);
	bb.setOffset(68);	bb.put(gps_oscillator_calibration);
	bb.setOffset(72);	bb.put(gps_code_select);
	bb.setOffset(76);	bb.put(gps_antenna_frequency_cmd);
	bb.setOffset(80);	bb.put(gps_svid_deselect_1_4);
	bb.setOffset(88);	bb.put(gps_svid_deselect_1_4_2);
	bb.setOffset(96);	bb.put(pre_launch_temperature_ramp);
	bb.setOffset(100);	bb.put(post_launch_temperature_ramp);
	bb.setOffset(104);	bb.put(post_launch_power_on_time_delay);
	bb.setOffset(108);	bb.put(post_launch_power_on_time_delay_uncertainty);
	Checksum = computeChecksum((uint32_t*)buffer, 44);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1003::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 176) return -2;

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
	bb.setOffset(16);	bb.get(anti_jam_selection);
	bb.setOffset(20);	bb.get(gps_antenna_spin);
	bb.setOffset(24);	bb.get(constellation_type);
	bb.setOffset(28);	bb.get(acquisition_state_selection);
	bb.setOffset(32);	bb.get(ionosphere_corrections);
	bb.setOffset(36);	bb.get(post_launch_auto_transition_to_trk);
	bb.setOffset(40);	bb.get(set_back_shock);
	bb.setOffset(44);	bb.get(position_uncertainty);
	bb.setOffset(48);	bb.get(velocity_uncertainty);
	bb.setOffset(52);	bb.get(time_uncertainty);
	bb.setOffset(60);	bb.get(number_of_acquisition_attempts);
	bb.setOffset(64);	bb.get(gps_oscillator_calibration_jitter);
	bb.setOffset(68);	bb.get(gps_oscillator_calibration);
	bb.setOffset(72);	bb.get(gps_code_select);
	bb.setOffset(76);	bb.get(gps_antenna_frequency_cmd);
	bb.setOffset(80);	bb.get(gps_svid_deselect_1_4);
	bb.setOffset(88);	bb.get(gps_svid_deselect_1_4_2);
	bb.setOffset(96);	bb.get(pre_launch_temperature_ramp);
	bb.setOffset(100);	bb.get(post_launch_temperature_ramp);
	bb.setOffset(104);	bb.get(post_launch_power_on_time_delay);
	bb.setOffset(108);	bb.get(post_launch_power_on_time_delay_uncertainty);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 44);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

