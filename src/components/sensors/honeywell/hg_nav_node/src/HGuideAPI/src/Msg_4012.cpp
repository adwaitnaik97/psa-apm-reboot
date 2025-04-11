#include <include/HGuideAPI.h>
#include <include/Msg_4012.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4012::AddressId;
const uint32_t Msg_4012::MessageId;
const uint32_t Msg_4012::MessageLength;

Msg_4012::Msg_4012()
{
	Default();
}

void Msg_4012::Default()
{
	Checksum = 0;
	nav_init_config.Default();
	Sigma_Latitude = 0;
	Sigma_Longitude = 0;
	Sigma_Altitude = 0;
	Sigma_Heading = 0;
}

bool Msg_4012::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 64) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint64_t nav_init_config_tmp = 0; // temporary variable holding the custom bitfield
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 2, nav_init_config.gyro_bias, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 3, nav_init_config.accel_bias, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 4, nav_init_config.gyro_sf, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 4, nav_init_config.accel_sf, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 6, nav_init_config.main_ant_lev, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 7, nav_init_config.aux_ant_lev, status_ok);
	nav_init_config_tmp = ECTOS::BIT_UTILITIES::PackBool(nav_init_config_tmp, 8, nav_init_config.nav_init, status_ok);
	bb.setOffset(16);	bb.put(nav_init_config_tmp);

	bb.setOffset(24);	bb.put(Sigma_Latitude);
	bb.setOffset(28);	bb.put(Sigma_Longitude);
	bb.setOffset(32);	bb.put(Sigma_Altitude);
	bb.setOffset(36);	bb.put(Sigma_Heading);
	Checksum = computeChecksum((uint32_t*)buffer, 16);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4012::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 64) return -2;

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

	uint64_t nav_init_config_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(nav_init_config_tmp);
	nav_init_config.gyro_bias = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 2,  status_ok);
	nav_init_config.accel_bias = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 3,  status_ok);
	nav_init_config.gyro_sf = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 4,  status_ok);
	nav_init_config.accel_sf = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 4,  status_ok);
	nav_init_config.main_ant_lev = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 6,  status_ok);
	nav_init_config.aux_ant_lev = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 7,  status_ok);
	nav_init_config.nav_init = ECTOS::BIT_UTILITIES::UnPackBool(nav_init_config_tmp, 8,  status_ok);
	bb.setOffset(24);	bb.get(Sigma_Latitude);
	bb.setOffset(28);	bb.get(Sigma_Longitude);
	bb.setOffset(32);	bb.get(Sigma_Altitude);
	bb.setOffset(36);	bb.get(Sigma_Heading);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 16);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

