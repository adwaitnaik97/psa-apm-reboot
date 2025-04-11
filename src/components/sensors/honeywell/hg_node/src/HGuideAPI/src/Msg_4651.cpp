#include <include/HGuideAPI.h>
#include <include/Msg_4651.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4651::AddressId;
const uint32_t Msg_4651::MessageId;
const uint32_t Msg_4651::MessageLength;

Msg_4651::Msg_4651()
{
	Default();
}

void Msg_4651::Default()
{
	Checksum = 0;
	WarmupTime = 0;
	ThresholdCheckEnable.Default();
	PosXErrorThreshold = 0;
	PosYErrorThreshold = 0;
	PosZErrorThreshold = 0;
	VelXErrorThreshold = 0;
	VelYErrorThreshold = 0;
	VelZErrorThreshold = 0;
	AttXErrorThreshold = 0;
	AttYErrorThreshold = 0;
	AttZErrorThreshold = 0;
	NavAxesSelector = 0;
}

bool Msg_4651::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 80) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(WarmupTime);

	uint32_t ThresholdCheckEnable_tmp = 0; // temporary variable holding the custom bitfield
	ThresholdCheckEnable_tmp = ECTOS::BIT_UTILITIES::PackBool(ThresholdCheckEnable_tmp, 0, ThresholdCheckEnable.position, status_ok);
	ThresholdCheckEnable_tmp = ECTOS::BIT_UTILITIES::PackBool(ThresholdCheckEnable_tmp, 1, ThresholdCheckEnable.velocity, status_ok);
	ThresholdCheckEnable_tmp = ECTOS::BIT_UTILITIES::PackBool(ThresholdCheckEnable_tmp, 2, ThresholdCheckEnable.attitude, status_ok);
	bb.setOffset(20);	bb.put(ThresholdCheckEnable_tmp);

	bb.setOffset(24);	bb.put(PosXErrorThreshold);
	bb.setOffset(28);	bb.put(PosYErrorThreshold);
	bb.setOffset(32);	bb.put(PosZErrorThreshold);
	bb.setOffset(36);	bb.put(VelXErrorThreshold);
	bb.setOffset(40);	bb.put(VelYErrorThreshold);
	bb.setOffset(44);	bb.put(VelZErrorThreshold);
	bb.setOffset(48);	bb.put(AttXErrorThreshold);
	bb.setOffset(52);	bb.put(AttYErrorThreshold);
	bb.setOffset(56);	bb.put(AttZErrorThreshold);
	bb.setOffset(60);	bb.put(NavAxesSelector);
	Checksum = computeChecksum((uint32_t*)buffer, 20);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4651::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 80) return -2;

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
	bb.setOffset(16);	bb.get(WarmupTime);

	uint32_t ThresholdCheckEnable_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(20);	bb.get(ThresholdCheckEnable_tmp);
	ThresholdCheckEnable.position = ECTOS::BIT_UTILITIES::UnPackBool(ThresholdCheckEnable_tmp, 0,  status_ok);
	ThresholdCheckEnable.velocity = ECTOS::BIT_UTILITIES::UnPackBool(ThresholdCheckEnable_tmp, 1,  status_ok);
	ThresholdCheckEnable.attitude = ECTOS::BIT_UTILITIES::UnPackBool(ThresholdCheckEnable_tmp, 2,  status_ok);
	bb.setOffset(24);	bb.get(PosXErrorThreshold);
	bb.setOffset(28);	bb.get(PosYErrorThreshold);
	bb.setOffset(32);	bb.get(PosZErrorThreshold);
	bb.setOffset(36);	bb.get(VelXErrorThreshold);
	bb.setOffset(40);	bb.get(VelYErrorThreshold);
	bb.setOffset(44);	bb.get(VelZErrorThreshold);
	bb.setOffset(48);	bb.get(AttXErrorThreshold);
	bb.setOffset(52);	bb.get(AttYErrorThreshold);
	bb.setOffset(56);	bb.get(AttZErrorThreshold);
	bb.setOffset(60);	bb.get(NavAxesSelector);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 20);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

