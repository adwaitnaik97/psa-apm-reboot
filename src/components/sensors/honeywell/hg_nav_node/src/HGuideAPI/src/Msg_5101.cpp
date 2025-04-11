#include <include/HGuideAPI.h>
#include <include/Msg_5101.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5101::AddressId;
const uint32_t Msg_5101::MessageId;

Msg_5101::Msg_5101()
{
	Default();
}

bool Msg_5101::set_observation_size(uint32_t size)
{
	if (max_observation_size > 0){
		free(observation);
	}
	if (0 == size){
		max_observation_size = size;
		return true;
	}
	observation = (observation_t*) malloc(sizeof(observation_t)*size);
	if (NULL != observation){
		for (unsigned int index=0;index<size;index++){
			observation[index].Default();
		}
		max_observation_size = size;
		return true;
	}
	return false;
}

uint32_t Msg_5101::get_observation_size(void)
{
	if (max_observation_size > Num_Observations){
		return Num_Observations;
	}
	else{
		return max_observation_size;
	}
}

void Msg_5101::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	Num_Observations = 0;
	for (unsigned int index = 0; index < max_observation_size; index++){
		observation[index].Default();
	}
	MessageLength = 40+Num_Observations*11;
}

bool Msg_5101::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < (40+Num_Observations*11)*4) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	// Update the value before serialization
	MessageLength = 40+Num_Observations*11;
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(GPSMode));
	bb.setOffset(36);	bb.put(Num_Observations);

	uint32_t observation_tmp[11] = { 0 }; // buffer holding the custom bitfield

	// Set the repeated block structure
	for (unsigned int i=0; i<((Num_Observations<=max_observation_size)?Num_Observations:max_observation_size);i++){
	observation_tmp[0] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[0], 0, 15, observation[i].prn_number, status_ok);
	observation_tmp[0] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[0], 16, 31, observation[i].glofreq, status_ok);
	*(double*)(observation_tmp + 1) = observation[i].pseudorange;
	*(float*)(observation_tmp + 3) = observation[i].pseudorange_stdv;
	*(double*)(observation_tmp + 4) = observation[i].carrier_phase;
	*(float*)(observation_tmp + 6) = observation[i].carrier_phase_stdv;
	*(float*)(observation_tmp + 7) = observation[i].doppler_freq;
	*(float*)(observation_tmp + 8) = observation[i].carrier_to_noise;
	*(float*)(observation_tmp + 9) = observation[i].locktime;
	observation_tmp[10] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[10], 0, 4, observation[i].tracking_state, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[10], 5, 9, observation[i].sv_channel_number, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 10, observation[i].phase_lock, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 11, observation[i].parity_known, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 12, observation[i].code_lock, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[10], 13, 15, observation[i].correlator_type, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[10], 16, 18, observation[i].system, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 20, observation[i].grouping, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::Pack(observation_tmp[10], 21, 25, observation[i].signal_type, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 27, observation[i].primary_L1_channel, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 28, observation[i].carrier_phase_meas, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 29, observation[i].digital_filter, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 30, observation[i].PRN_lock_flag, status_ok);
	observation_tmp[10] = ECTOS::BIT_UTILITIES::PackBool(observation_tmp[10], 31, observation[i].channel_assignment, status_ok);
	bb.setOffset(40+i*44.0);	bb.put(observation_tmp);

	}

	Checksum = computeChecksum((uint32_t*)buffer, 40+Num_Observations*11);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5101::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
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
	bb.setOffset(8);	bb.get(MessageLength);
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);	bb.get(gpsTov);
	bb.setOffset(32);	GPSMode = static_cast<gps_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(36);	bb.get(Num_Observations);

	uint32_t observation_tmp[11] = { 0 }; // buffer holding the custom bitfield

	// Get the repeated block structure
	for (unsigned int i=0; i<((Num_Observations<=max_observation_size)?Num_Observations:max_observation_size);i++){
	bb.setOffset(40+i*44.0);	bb.get(observation_tmp);
	observation[i].prn_number = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[0], 0, 15,  status_ok);
	observation[i].glofreq = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[0], 16, 31,  status_ok);
	memcpy(&observation[i].pseudorange,observation_tmp+1,sizeof(double));
	memcpy(&observation[i].pseudorange_stdv,observation_tmp+3,sizeof(float));
	memcpy(&observation[i].carrier_phase,observation_tmp+4,sizeof(double));
	memcpy(&observation[i].carrier_phase_stdv,observation_tmp+6,sizeof(float));
	memcpy(&observation[i].doppler_freq,observation_tmp+7,sizeof(float));
	memcpy(&observation[i].carrier_to_noise,observation_tmp+8,sizeof(float));
	memcpy(&observation[i].locktime,observation_tmp+9,sizeof(float));
	observation[i].tracking_state = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[10], 0, 4,  status_ok);
	observation[i].sv_channel_number = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[10], 5, 9,  status_ok);
	observation[i].phase_lock = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 10,  status_ok);
	observation[i].parity_known = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 11,  status_ok);
	observation[i].code_lock = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 12,  status_ok);
	observation[i].correlator_type = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[10], 13, 15,  status_ok);
	observation[i].system = static_cast<constellation_enum_t>(ECTOS::BIT_UTILITIES::UnPack(observation_tmp[10], 16, 18,  status_ok));
	observation[i].grouping = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 20,  status_ok);
	observation[i].signal_type = ECTOS::BIT_UTILITIES::UnPack(observation_tmp[10], 21, 25,  status_ok);
	observation[i].primary_L1_channel = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 27,  status_ok);
	observation[i].carrier_phase_meas = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 28,  status_ok);
	observation[i].digital_filter = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 29,  status_ok);
	observation[i].PRN_lock_flag = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 30,  status_ok);
	observation[i].channel_assignment = ECTOS::BIT_UTILITIES::UnPackBool(observation_tmp[10], 31,  status_ok);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 40+Num_Observations*11);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

