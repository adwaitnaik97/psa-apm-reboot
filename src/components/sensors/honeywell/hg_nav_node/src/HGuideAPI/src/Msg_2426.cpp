#include <include/HGuideAPI.h>
#include <include/Msg_2426.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2426::AddressId;
const uint32_t Msg_2426::MessageId;
const uint32_t Msg_2426::MessageLength;

Msg_2426::Msg_2426()
{
	Default();
}

void Msg_2426::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	BaroBiasErrEst = 0;
	BaroScaleFactorErrEst = 0;
	BaroBiasSTDV = 0;
	BaroScaleFactorSTDV = 0;
	BaroMeasResid = 0;
}

bool Msg_2426::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 84) return false;

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
	bb.setOffset(44);	bb.put(BaroBiasErrEst);
	bb.setOffset(48);	bb.put(BaroScaleFactorErrEst);
	bb.setOffset(64);	bb.put(BaroBiasSTDV);
	bb.setOffset(68);	bb.put(BaroScaleFactorSTDV);
	bb.setOffset(76);	bb.put(BaroMeasResid);
	Checksum = computeChecksum((uint32_t*)buffer, 21);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2426::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 84) return -2;

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
	bb.setOffset(44);	bb.get(BaroBiasErrEst);
	bb.setOffset(48);	bb.get(BaroScaleFactorErrEst);
	bb.setOffset(64);	bb.get(BaroBiasSTDV);
	bb.setOffset(68);	bb.get(BaroScaleFactorSTDV);
	bb.setOffset(76);	bb.get(BaroMeasResid);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 21);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

