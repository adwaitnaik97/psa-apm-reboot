#include <include/HGuideAPI.h>
#include <include/Msg_FB.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_FB::SyncByte;
const uint8_t Msg_FB::MessageID;

Msg_FB::Msg_FB()
{
	Default();
}

void Msg_FB::Default()
{
	crc_fail = 0;
	hardware_fail = 0;
	Checksum = 0;
}

bool Msg_FB::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(crc_fail);
	bb.setOffset(4);	bb.put(hardware_fail);
	int cIndex = 10;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 12);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_FB::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(2);	bb.get(crc_fail);
	bb.setOffset(4);	bb.get(hardware_fail);
	bb.setOffset(10);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 12);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

