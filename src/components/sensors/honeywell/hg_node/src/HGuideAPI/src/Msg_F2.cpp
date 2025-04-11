#include <include/HGuideAPI.h>
#include <include/Msg_F2.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_F2::SyncByte;
const uint8_t Msg_F2::MessageID;

Msg_F2::Msg_F2()
{
	Default();
}

void Msg_F2::Default()
{
	part_number = 0;
	for (unsigned int index = 0; index < 4; index++)
	{
		hardware_version[index] = 0;
	}
	Checksum = 0;
}

bool Msg_F2::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(part_number);
	bb.setOffset(6);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.put(hardware_version[index]);
	}
	int cIndex = 10;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 12);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_F2::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint8_t SyncByte_In;
	bb.setOffset(0);	bb.get(SyncByte_In);
	constCheck |= (SyncByte != SyncByte_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint8_t MessageID_In;
	bb.setOffset(1);	bb.get(MessageID_In);
	constCheck |= (MessageID != MessageID_In) << numConsts++;
	bb.setOffset(2);	bb.get(part_number);
	bb.setOffset(6);
	for (unsigned int index = 0; index < 4; index++)
	{
		bb.get(hardware_version[index]);
	}
	bb.setOffset(10);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 12);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

