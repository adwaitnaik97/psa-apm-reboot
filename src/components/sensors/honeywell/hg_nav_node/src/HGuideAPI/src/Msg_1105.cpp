#include <include/HGuideAPI.h>
#include <include/Msg_1105.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1105::AddressId;
const uint32_t Msg_1105::MessageId;
const uint32_t Msg_1105::MessageLength;

Msg_1105::Msg_1105()
{
	Default();
}

void Msg_1105::Default()
{
	Checksum = 0;
	MagneticHeadingTov = 0;
	MagneticHeadingValidity = 0;
	TimeReferenceMode = 0;
	MagneticVariationValidity = 0;
	MagneticHeading = 0;
	MagneticVariation = 0;
}

bool Msg_1105::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 40) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(MagneticHeadingTov);

	uint8_t bitfieldAtByte24 = 0; // temporary variable holding the bitfield
	bitfieldAtByte24 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte24, 0, MagneticHeadingValidity, status_ok);
	bitfieldAtByte24 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte24, 1, TimeReferenceMode, status_ok);
	bitfieldAtByte24 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte24, 2, MagneticVariationValidity, status_ok);
	bb.setOffset(24);	bb.put(bitfieldAtByte24);

	bb.setOffset(28);	bb.put(MagneticHeading);
	bb.setOffset(32);	bb.put(MagneticVariation);
	Checksum = computeChecksum((uint32_t*)buffer, 10);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1105::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 40) return -2;

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
	bb.setOffset(16);	bb.get(MagneticHeadingTov);

	uint8_t bitfieldAtByte24 = 0; // temporary variable holding the bitfield
	bb.setOffset(24);	bb.get(bitfieldAtByte24);
	MagneticHeadingValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte24, 0,  status_ok);
	TimeReferenceMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte24, 1,  status_ok);
	MagneticVariationValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte24, 2,  status_ok);
	bb.setOffset(28);	bb.get(MagneticHeading);
	bb.setOffset(32);	bb.get(MagneticVariation);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 10);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

