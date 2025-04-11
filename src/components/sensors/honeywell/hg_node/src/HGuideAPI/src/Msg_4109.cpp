#include <include/HGuideAPI.h>
#include <include/Msg_4109.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4109::AddressId;
const uint32_t Msg_4109::MessageId;
const uint32_t Msg_4109::MessageLength;

Msg_4109::Msg_4109()
{
	Default();
}

void Msg_4109::Default()
{
	Checksum = 0;
	RequestAckNak = 0;
	RollValid = 0;
	PitchValid = 0;
	HeadingValid = 0;
	TovMode = 0;
	MessageTov = 0;
	Roll = 0;
	Pitch = 0;
	TrueHdg = 0;
	RollStdv = 0;
	PitchStdv = 0;
	TrueHdgStdv = 0;
	Baseline = 0;
}

bool Msg_4109::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 60) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 0, RequestAckNak, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 1, RollValid, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 2, PitchValid, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 3, HeadingValid, status_ok);
	bb.setOffset(16);	bb.put(bitfieldAtByte16);


	uint32_t bitfieldAtByte20 = 0; // temporary variable holding the bitfield
	bitfieldAtByte20 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte20, 0, TovMode, status_ok);
	bb.setOffset(20);	bb.put(bitfieldAtByte20);

	bb.setOffset(24);	bb.put(MessageTov);
	bb.setOffset(32);	bb.put(Roll);
	bb.setOffset(36);	bb.put(Pitch);
	bb.setOffset(40);	bb.put(TrueHdg);
	bb.setOffset(44);	bb.put(RollStdv);
	bb.setOffset(48);	bb.put(PitchStdv);
	bb.setOffset(52);	bb.put(TrueHdgStdv);
	bb.setOffset(56);	bb.put(Baseline);
	Checksum = computeChecksum((uint32_t*)buffer, 15);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4109::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 60) return -2;

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

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bb.setOffset(16);	bb.get(bitfieldAtByte16);
	RequestAckNak = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 0,  status_ok);
	RollValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 1,  status_ok);
	PitchValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 2,  status_ok);
	HeadingValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 3,  status_ok);

	uint32_t bitfieldAtByte20 = 0; // temporary variable holding the bitfield
	bb.setOffset(20);	bb.get(bitfieldAtByte20);
	TovMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte20, 0,  status_ok);
	bb.setOffset(24);	bb.get(MessageTov);
	bb.setOffset(32);	bb.get(Roll);
	bb.setOffset(36);	bb.get(Pitch);
	bb.setOffset(40);	bb.get(TrueHdg);
	bb.setOffset(44);	bb.get(RollStdv);
	bb.setOffset(48);	bb.get(PitchStdv);
	bb.setOffset(52);	bb.get(TrueHdgStdv);
	bb.setOffset(56);	bb.get(Baseline);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 15);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

