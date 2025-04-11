#include <include/HGuideAPI.h>
#include <include/Msg_F6.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_F6::SyncByte;
const uint8_t Msg_F6::MessageID;

Msg_F6::Msg_F6()
{
	Default();
}

void Msg_F6::Default()
{
	accel_range = 0;
	accel_config.Default();
	accel_sample_rate = 0;
	Checksum = 0;
}

bool Msg_F6::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(accel_range);

	uint16_t accel_config_tmp = 0; // temporary variable holding the custom bitfield
	accel_config_tmp = ECTOS::BIT_UTILITIES::Pack(accel_config_tmp, 0, 14, static_cast<uint16_t>CHECK_MAX_UINT16(accel_config.cutoff_frequency / (3.051850947599719e-05)), status_ok);
	accel_config_tmp = ECTOS::BIT_UTILITIES::PackBool(accel_config_tmp, 15, accel_config.enabled, status_ok);
	bb.setOffset(6);	bb.put(accel_config_tmp);

	bb.setOffset(8);	bb.put(accel_sample_rate);
	int cIndex = 10;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 12);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_F6::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 12) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint8_t SyncByte_In;
	bb.setOffset(0);	bb.get(SyncByte_In);
	constCheck |= (SyncByte != SyncByte_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint8_t MessageID_In;
	bb.setOffset(1);	bb.get(MessageID_In);
	constCheck |= (MessageID != MessageID_In) << numConsts++;
	bb.setOffset(2);	bb.get(accel_range);

	uint16_t accel_config_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(6);	bb.get(accel_config_tmp);
	accel_config.cutoff_frequency = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(accel_config_tmp, 0, 14,  status_ok) * (3.051850947599719e-05));
	accel_config.enabled = ECTOS::BIT_UTILITIES::UnPackBool(accel_config_tmp, 15,  status_ok);
	bb.setOffset(8);	bb.get(accel_sample_rate);
	bb.setOffset(10);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 12);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

