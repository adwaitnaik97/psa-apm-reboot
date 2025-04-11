#include <include/HGuideAPI.h>
#include <include/Msg_6724.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6724::AddressId;
const uint32_t Msg_6724::MessageId;
const uint32_t Msg_6724::MessageLength;

Msg_6724::Msg_6724()
{
	Default();
}

void Msg_6724::Default()
{
	Checksum = 0;
	systemTov = 0;
	for (unsigned int index = 0; index < 30; index++)
	{
		velBeam0[index] = 0;
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		velBeam1[index] = 0;
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		velBeam2[index] = 0;
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		velBeam3[index] = 0;
	}
}

bool Msg_6724::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 264) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.put(velBeam0[index]);
	}
	bb.setOffset(84);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.put(velBeam1[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.put(velBeam2[index]);
	}
	bb.setOffset(204);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.put(velBeam3[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 66);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6724::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 264) return -2;

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
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.get(velBeam0[index]);
	}
	bb.setOffset(84);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.get(velBeam1[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.get(velBeam2[index]);
	}
	bb.setOffset(204);
	for (unsigned int index = 0; index < 30; index++)
	{
		bb.get(velBeam3[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 66);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

