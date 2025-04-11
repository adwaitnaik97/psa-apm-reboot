#include <include/HGuideAPI.h>
#include <include/Msg_5102.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5102::AddressId;
const uint32_t Msg_5102::MessageId;

Msg_5102::Msg_5102()
{
	Default();
}

bool Msg_5102::set_sat_position_size(uint32_t size)
{
	if (max_sat_position_size > 0){
		free(sat_position);
	}
	if (0 == size){
		max_sat_position_size = size;
		return true;
	}
	sat_position = (satellite_pos_t*) malloc(sizeof(satellite_pos_t)*size);
	if (NULL != sat_position){
		for (unsigned int index=0;index<size;index++){
			sat_position[index].Default();
		}
		max_sat_position_size = size;
		return true;
	}
	return false;
}

uint32_t Msg_5102::get_sat_position_size(void)
{
	if (max_sat_position_size > num_satellites){
		return num_satellites;
	}
	else{
		return max_sat_position_size;
	}
}

void Msg_5102::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
	num_satellites = 0;
	total_pages = 1;
	page_index = 0;
	for (unsigned int index = 0; index < max_sat_position_size; index++){
		sat_position[index].Default();
	}
	MessageLength = 11+num_satellites*18;
}

bool Msg_5102::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < (11+num_satellites*18)*4) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	// Update the value before serialization
	MessageLength = 11+num_satellites*18;
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(gps_week);
	bb.setOffset(36);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(GPSMode));
	bb.setOffset(40);	bb.put(num_satellites);
	bb.setOffset(41);	bb.put(total_pages);
	bb.setOffset(42);	bb.put(page_index);

	uint32_t sat_position_tmp[18] = { 0 }; // buffer holding the custom bitfield

	// Set the repeated block structure
	for (unsigned int i=0; i<((num_satellites<=max_sat_position_size)?num_satellites:max_sat_position_size);i++){
	sat_position_tmp[0] = ECTOS::BIT_UTILITIES::Pack(sat_position_tmp[0], 0, 31, sat_position[i].system, status_ok);
	sat_position_tmp[1] = ECTOS::BIT_UTILITIES::Pack(sat_position_tmp[1], 0, 31, sat_position[i].satelliteID, status_ok);
	*(double*)(sat_position_tmp + 2) = sat_position[i].pos_x;
	*(double*)(sat_position_tmp + 4) = sat_position[i].pos_y;
	*(double*)(sat_position_tmp + 6) = sat_position[i].pos_z;
	*(double*)(sat_position_tmp + 8) = sat_position[i].clk_corr;
	*(double*)(sat_position_tmp + 10) = sat_position[i].iono_delay;
	*(double*)(sat_position_tmp + 12) = sat_position[i].tropo_delay;
	bb.setOffset(44+i*72.0);	bb.put(sat_position_tmp);

	}

	Checksum = computeChecksum((uint32_t*)buffer, 11+num_satellites*18);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5102::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(32);	bb.get(gps_week);
	bb.setOffset(36);	GPSMode = static_cast<gps_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(40);	bb.get(num_satellites);
	bb.setOffset(41);	bb.get(total_pages);
	bb.setOffset(42);	bb.get(page_index);

	uint32_t sat_position_tmp[18] = { 0 }; // buffer holding the custom bitfield

	// Get the repeated block structure
	for (unsigned int i=0; i<((num_satellites<=max_sat_position_size)?num_satellites:max_sat_position_size);i++){
	bb.setOffset(44+i*72.0);	bb.get(sat_position_tmp);
	sat_position[i].system= static_cast<constellation_enum_t>(*(uint32_t*)(sat_position_tmp+0));
	sat_position[i].satelliteID = ECTOS::BIT_UTILITIES::UnPack(sat_position_tmp[1], 0, 31,  status_ok);
	memcpy(&sat_position[i].pos_x,sat_position_tmp+2,sizeof(double));
	memcpy(&sat_position[i].pos_y,sat_position_tmp+4,sizeof(double));
	memcpy(&sat_position[i].pos_z,sat_position_tmp+6,sizeof(double));
	memcpy(&sat_position[i].clk_corr,sat_position_tmp+8,sizeof(double));
	memcpy(&sat_position[i].iono_delay,sat_position_tmp+10,sizeof(double));
	memcpy(&sat_position[i].tropo_delay,sat_position_tmp+12,sizeof(double));
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 11+num_satellites*18);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

