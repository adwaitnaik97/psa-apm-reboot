#include <include/HGuideAPI.h>
#include <include/Msg_20FF.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_20FF::AddressId;
const uint32_t Msg_20FF::MessageId;
const uint32_t Msg_20FF::MessageLength;

Msg_20FF::Msg_20FF()
{
	Default();
}

void Msg_20FF::Default()
{
	Checksum = 0;
	Ack = 0;
	InputMessageID = 0;
	NoOfValidMessagesSinceLast = 0;
	NoOfValidMessagesSincePowerUp = 0;
	MessageTimeOfReception = 0;
}

bool Msg_20FF::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(Ack);
	bb.setOffset(20);	bb.put(InputMessageID);
	bb.setOffset(24);	bb.put(NoOfValidMessagesSinceLast);
	bb.setOffset(28);	bb.put(NoOfValidMessagesSincePowerUp);
	bb.setOffset(32);	bb.put(MessageTimeOfReception);
	Checksum = computeChecksum((uint32_t*)buffer, 11);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_20FF::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return -2;

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
	bb.setOffset(16);	bb.get(Ack);
	bb.setOffset(20);	bb.get(InputMessageID);
	bb.setOffset(24);	bb.get(NoOfValidMessagesSinceLast);
	bb.setOffset(28);	bb.get(NoOfValidMessagesSincePowerUp);
	bb.setOffset(32);	bb.get(MessageTimeOfReception);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 11);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

