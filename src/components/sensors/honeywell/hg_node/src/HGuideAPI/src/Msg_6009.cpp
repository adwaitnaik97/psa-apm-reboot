#include <include/HGuideAPI.h>
#include <include/Msg_6009.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6009::AddressId;
const uint32_t Msg_6009::MessageId;
const uint32_t Msg_6009::MessageLength;

Msg_6009::Msg_6009()
{
	Default();
}

void Msg_6009::Default()
{
	Checksum = 0;
	for (unsigned int index = 0; index < 128; index++)
	{
		casterHostName[index] = 0;
	}
	casterIP_Port = 0;
	casterProtocol = 0;
	ggaRequired = 0;
	ggaOutputInterval = 0;
	for (unsigned int index = 0; index < 100; index++)
	{
		mountPointStr[index] = 0;
	}
	for (unsigned int index = 0; index < 100; index++)
	{
		userAgentStr[index] = 0;
	}
	for (unsigned int index = 0; index < 20; index++)
	{
		userAgentVerStr[index] = 0;
	}
	for (unsigned int index = 0; index < 50; index++)
	{
		userIDStr[index] = 0;
	}
	for (unsigned int index = 0; index < 50; index++)
	{
		userPasswdStr[index] = 0;
	}
}

bool Msg_6009::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 480) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);
	for (unsigned int index = 0; index < 128; index++)
	{
		bb.put(casterHostName[index]);
	}
	bb.setOffset(144);	bb.put(casterIP_Port);
	bb.setOffset(148);	bb.put(casterProtocol);

	uint32_t bitfieldAtByte152 = 0; // temporary variable holding the bitfield
	bitfieldAtByte152 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte152, 0, ggaRequired, status_ok);
	bb.setOffset(152);	bb.put(bitfieldAtByte152);

	bb.setOffset(156);	bb.put(ggaOutputInterval);
	bb.setOffset(160);
	for (unsigned int index = 0; index < 100; index++)
	{
		bb.put(mountPointStr[index]);
	}
	bb.setOffset(260);
	for (unsigned int index = 0; index < 100; index++)
	{
		bb.put(userAgentStr[index]);
	}
	bb.setOffset(360);
	for (unsigned int index = 0; index < 20; index++)
	{
		bb.put(userAgentVerStr[index]);
	}
	bb.setOffset(380);
	for (unsigned int index = 0; index < 50; index++)
	{
		bb.put(userIDStr[index]);
	}
	bb.setOffset(430);
	for (unsigned int index = 0; index < 50; index++)
	{
		bb.put(userPasswdStr[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 120);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6009::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 480) return -2;

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
	bb.setOffset(16);
	for (unsigned int index = 0; index < 128; index++)
	{
		bb.get(casterHostName[index]);
	}
	bb.setOffset(144);	bb.get(casterIP_Port);
	bb.setOffset(148);	bb.get(casterProtocol);

	uint32_t bitfieldAtByte152 = 0; // temporary variable holding the bitfield
	bb.setOffset(152);	bb.get(bitfieldAtByte152);
	ggaRequired = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte152, 0,  status_ok);
	bb.setOffset(156);	bb.get(ggaOutputInterval);
	bb.setOffset(160);
	for (unsigned int index = 0; index < 100; index++)
	{
		bb.get(mountPointStr[index]);
	}
	bb.setOffset(260);
	for (unsigned int index = 0; index < 100; index++)
	{
		bb.get(userAgentStr[index]);
	}
	bb.setOffset(360);
	for (unsigned int index = 0; index < 20; index++)
	{
		bb.get(userAgentVerStr[index]);
	}
	bb.setOffset(380);
	for (unsigned int index = 0; index < 50; index++)
	{
		bb.get(userIDStr[index]);
	}
	bb.setOffset(430);
	for (unsigned int index = 0; index < 50; index++)
	{
		bb.get(userPasswdStr[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 120);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

