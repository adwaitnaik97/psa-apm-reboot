#include <include/HGuideAPI.h>
#include <include/Msg_6001.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6001::AddressId;
const uint32_t Msg_6001::MessageId;
const uint32_t Msg_6001::MessageLength;

Msg_6001::Msg_6001()
{
	Default();
}

void Msg_6001::Default()
{
	Checksum = 0;
	for (unsigned int index = 0; index < 16; index++)
	{
		DeviceSerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		DevicePartNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		SensorAssyPartNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		NavSoftwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		NavSoftwareBuildDate[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		IMUSerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		IMUPartNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		IMUSoftwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		GNSSReceiverSerialNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		GNSSReceiverPartNumber[index] = 0;
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		GNSSReceiverFirmwareVersion[index] = 0;
	}
	for (unsigned int index = 0; index < 32; index++)
	{
		ProcessorHWIdentifier[index] = 0;
	}
	for (unsigned int index = 0; index < 32; index++)
	{
		InterconnectHWIdentifier[index] = 0;
	}
}

bool Msg_6001::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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
		bb.put(DeviceSerialNumber[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(DevicePartNumber[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(SensorAssyPartNumber[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(NavSoftwareVersion[index]);
	}
	bb.setOffset(80);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(NavSoftwareBuildDate[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(IMUSerialNumber[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(IMUPartNumber[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(IMUSoftwareVersion[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(GNSSReceiverSerialNumber[index]);
	}
	bb.setOffset(160);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(GNSSReceiverPartNumber[index]);
	}
	bb.setOffset(176);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.put(GNSSReceiverFirmwareVersion[index]);
	}
	bb.setOffset(192);
	for (unsigned int index = 0; index < 32; index++)
	{
		bb.put(ProcessorHWIdentifier[index]);
	}
	bb.setOffset(224);
	for (unsigned int index = 0; index < 32; index++)
	{
		bb.put(InterconnectHWIdentifier[index]);
	}
	Checksum = computeChecksum((uint32_t*)buffer, 64);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6001::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
		bb.get(DeviceSerialNumber[index]);
	}
	bb.setOffset(32);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(DevicePartNumber[index]);
	}
	bb.setOffset(48);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(SensorAssyPartNumber[index]);
	}
	bb.setOffset(64);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(NavSoftwareVersion[index]);
	}
	bb.setOffset(80);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(NavSoftwareBuildDate[index]);
	}
	bb.setOffset(96);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(IMUSerialNumber[index]);
	}
	bb.setOffset(112);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(IMUPartNumber[index]);
	}
	bb.setOffset(128);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(IMUSoftwareVersion[index]);
	}
	bb.setOffset(144);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(GNSSReceiverSerialNumber[index]);
	}
	bb.setOffset(160);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(GNSSReceiverPartNumber[index]);
	}
	bb.setOffset(176);
	for (unsigned int index = 0; index < 16; index++)
	{
		bb.get(GNSSReceiverFirmwareVersion[index]);
	}
	bb.setOffset(192);
	for (unsigned int index = 0; index < 32; index++)
	{
		bb.get(ProcessorHWIdentifier[index]);
	}
	bb.setOffset(224);
	for (unsigned int index = 0; index < 32; index++)
	{
		bb.get(InterconnectHWIdentifier[index]);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 64);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

