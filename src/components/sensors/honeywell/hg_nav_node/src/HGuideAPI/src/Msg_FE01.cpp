#include <include/HGuideAPI.h>
#include <include/Msg_FE01.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_FE01::AddressId;
const uint32_t Msg_FE01::MessageId;
const uint32_t Msg_FE01::MessageLength;

Msg_FE01::Msg_FE01()
{
	Default();
}

void Msg_FE01::Default()
{
	Checksum = 0;
	Parameter_1 = static_cast<msg_FE01_parameter_1_t>(0);
	Parameter_2 = 0;
	Parameter_3 = 0;
	Parameter_4 = 0;
	for (unsigned int index = 0; index < 248; index++)
	{
		Payload[index] = 0;
	}
}

bool Msg_FE01::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 280) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(Parameter_1));
	bb.setOffset(20);	bb.put(Parameter_2);
	bb.setOffset(24);	bb.put(Parameter_3);
	bb.setOffset(28);	bb.put(Parameter_4);
	bb.setOffset(32);
	for (unsigned int index = 0; index < 248; index++)
	{
		bb.put(Payload[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 70);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_FE01::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 280) return -2;

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
	bb.setOffset(16);	Parameter_1 = static_cast<msg_FE01_parameter_1_t>(bb.get<uint32_t>());
	bb.setOffset(20);	bb.get(Parameter_2);
	bb.setOffset(24);	bb.get(Parameter_3);
	bb.setOffset(28);	bb.get(Parameter_4);
	bb.setOffset(32);
	for (unsigned int index = 0; index < 248; index++)
	{
		bb.get(Payload[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 70);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

