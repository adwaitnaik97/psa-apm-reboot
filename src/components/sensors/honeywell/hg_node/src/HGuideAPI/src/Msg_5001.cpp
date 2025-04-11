#include <include/HGuideAPI.h>
#include <include/Msg_5001.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5001::AddressId;
const uint32_t Msg_5001::MessageId;
const uint32_t Msg_5001::MessageLength;

Msg_5001::Msg_5001()
{
	Default();
}

void Msg_5001::Default()
{
	Checksum = 0;
	for (unsigned int index = 0; index < 16; index++)
	{
		SerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		PartNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Model[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		GNSSHardwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		FirmwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		FirmwareBuildDate[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		FirmwareBuildTime[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component1Type[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component1Version[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component2Type[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component2Version[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component3Type[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		Component3Version[index] = 0;
	}
}

bool Msg_5001::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 256) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(SerialNumber[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(PartNumber[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Model[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(GNSSHardwareVersion[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(FirmwareVersion[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(FirmwareBuildDate[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(FirmwareBuildTime[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component1Type[index]);
	}
	bb.setOffset(160);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component1Version[index]);
	}
	bb.setOffset(176);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component2Type[index]);
	}
	bb.setOffset(192);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component2Version[index]);
	}
	bb.setOffset(208);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component3Type[index]);
	}
	bb.setOffset(224);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(Component3Version[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 64);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5001::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 256) return -2;

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
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(SerialNumber[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(PartNumber[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Model[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(GNSSHardwareVersion[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(FirmwareVersion[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(FirmwareBuildDate[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(FirmwareBuildTime[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component1Type[index]);
	}
	bb.setOffset(160);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component1Version[index]);
	}
	bb.setOffset(176);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component2Type[index]);
	}
	bb.setOffset(192);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component2Version[index]);
	}
	bb.setOffset(208);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component3Type[index]);
	}
	bb.setOffset(224);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(Component3Version[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 64);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

