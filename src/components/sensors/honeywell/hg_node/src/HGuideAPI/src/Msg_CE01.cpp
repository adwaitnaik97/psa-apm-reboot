#include <include/HGuideAPI.h>
#include <include/Msg_CE01.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_CE01::AddressId;
const uint32_t Msg_CE01::MessageId;
const uint32_t Msg_CE01::MessageLength;

Msg_CE01::Msg_CE01()
{
	Default();
}

void Msg_CE01::Default()
{
	Checksum = 0;
	PortId = 0;
	ASCII_String_Length = 0;
	for (unsigned int index = 0; index < 360; index++)
	{
		ASCII_String[index] = 0;
	}
}

bool Msg_CE01::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 388) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(PortId);
	bb.setOffset(20);	bb.put(ASCII_String_Length);
	bb.setOffset(24);
	for (unsigned int index = 0; index < 360; index++)
	{
		bb.put(ASCII_String[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 97);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_CE01::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 388) return -2;

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
	bb.setOffset(16);	bb.get(PortId);
	bb.setOffset(20);	bb.get(ASCII_String_Length);
	bb.setOffset(24);
	for (unsigned int index = 0; index < 360; index++)
	{
		bb.get(ASCII_String[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 97);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

