#include <include/HGuideAPI.h>
#include <include/Msg_4204.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4204::AddressId;
const uint32_t Msg_4204::MessageId;
const uint32_t Msg_4204::MessageLength;

Msg_4204::Msg_4204()
{
	Default();
}

void Msg_4204::Default()
{
	Checksum = 0;
	RF1AntennaX = 0;
	RF1AntennaY = 0;
	RF1AntennaZ = 0;
	RF2AntennaX = 0;
	RF2AntennaY = 0;
	RF2AntennaZ = 0;
	RF1AntennaSTDV = 0;
	RF2AntennaSTDV = 0;
	ChangeRF1LeverArm = 0;
	ChangeRF2LeverArm = 0;
	ChangeRF1STDV = 0;
	ChangeRF2STDV = 0;
}

bool Msg_4204::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 52) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(RF1AntennaX);
	bb.setOffset(20);	bb.put(RF1AntennaY);
	bb.setOffset(24);	bb.put(RF1AntennaZ);
	bb.setOffset(28);	bb.put(RF2AntennaX);
	bb.setOffset(32);	bb.put(RF2AntennaY);
	bb.setOffset(36);	bb.put(RF2AntennaZ);
	bb.setOffset(40);	bb.put(RF1AntennaSTDV);
	bb.setOffset(44);	bb.put(RF2AntennaSTDV);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 0, ChangeRF1LeverArm, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 1, ChangeRF2LeverArm, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 2, ChangeRF1STDV, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 3, ChangeRF2STDV, status_ok);
	bb.setOffset(48);	bb.put(bitfieldAtByte48);

	Checksum = computeChecksum((uint32_t*)buffer, 13);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4204::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 52) return -2;

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
	bb.setOffset(16);	bb.get(RF1AntennaX);
	bb.setOffset(20);	bb.get(RF1AntennaY);
	bb.setOffset(24);	bb.get(RF1AntennaZ);
	bb.setOffset(28);	bb.get(RF2AntennaX);
	bb.setOffset(32);	bb.get(RF2AntennaY);
	bb.setOffset(36);	bb.get(RF2AntennaZ);
	bb.setOffset(40);	bb.get(RF1AntennaSTDV);
	bb.setOffset(44);	bb.get(RF2AntennaSTDV);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bb.setOffset(48);	bb.get(bitfieldAtByte48);
	ChangeRF1LeverArm = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 0,  status_ok);
	ChangeRF2LeverArm = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 1,  status_ok);
	ChangeRF1STDV = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 2,  status_ok);
	ChangeRF2STDV = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 3,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 13);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

