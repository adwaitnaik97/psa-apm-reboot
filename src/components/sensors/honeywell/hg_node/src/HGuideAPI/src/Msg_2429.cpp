#include <include/HGuideAPI.h>
#include <include/Msg_2429.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2429::AddressId;
const uint32_t Msg_2429::MessageId;
const uint32_t Msg_2429::MessageLength;

Msg_2429::Msg_2429()
{
	Default();
}

void Msg_2429::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	TA_PositionX_MeasResid = 0;
	TA_PositionY_MeasResid = 0;
	TA_PositionZ_MeasResid = 0;
	TA_VelocityX_MeasResid = 0;
	TA_VelocityY_MeasResid = 0;
	TA_VelocityZ_MeasResid = 0;
	TA_AttitudeX_MeasResid = 0;
	TA_AttitudeY_MeasResid = 0;
	TA_AttitudeZ_MeasResid = 0;
	TA_AttitudeCosZ_MeasResid = 0;
}

bool Msg_2429::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 96) return false;

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
	bb.setOffset(44);	bb.put(TA_PositionX_MeasResid);
	bb.setOffset(48);	bb.put(TA_PositionY_MeasResid);
	bb.setOffset(52);	bb.put(TA_PositionZ_MeasResid);
	bb.setOffset(56);	bb.put(TA_VelocityX_MeasResid);
	bb.setOffset(60);	bb.put(TA_VelocityY_MeasResid);
	bb.setOffset(64);	bb.put(TA_VelocityZ_MeasResid);
	bb.setOffset(68);	bb.put(TA_AttitudeX_MeasResid);
	bb.setOffset(72);	bb.put(TA_AttitudeY_MeasResid);
	bb.setOffset(76);	bb.put(TA_AttitudeZ_MeasResid);
	bb.setOffset(80);	bb.put(TA_AttitudeCosZ_MeasResid);
	Checksum = computeChecksum((uint32_t*)buffer, 24);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2429::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 96) return -2;

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
	bb.setOffset(44);	bb.get(TA_PositionX_MeasResid);
	bb.setOffset(48);	bb.get(TA_PositionY_MeasResid);
	bb.setOffset(52);	bb.get(TA_PositionZ_MeasResid);
	bb.setOffset(56);	bb.get(TA_VelocityX_MeasResid);
	bb.setOffset(60);	bb.get(TA_VelocityY_MeasResid);
	bb.setOffset(64);	bb.get(TA_VelocityZ_MeasResid);
	bb.setOffset(68);	bb.get(TA_AttitudeX_MeasResid);
	bb.setOffset(72);	bb.get(TA_AttitudeY_MeasResid);
	bb.setOffset(76);	bb.get(TA_AttitudeZ_MeasResid);
	bb.setOffset(80);	bb.get(TA_AttitudeCosZ_MeasResid);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 24);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

