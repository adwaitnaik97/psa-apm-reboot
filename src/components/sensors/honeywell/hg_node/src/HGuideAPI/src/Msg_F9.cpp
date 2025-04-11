#include <include/HGuideAPI.h>
#include <include/Msg_F9.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_F9::SyncByte;
const uint8_t Msg_F9::MessageID;

Msg_F9::Msg_F9()
{
	Default();
}

void Msg_F9::Default()
{
	percent_fr = 0;
	percent_ffr = 0;
	percent_cr = 0;
	percent_gr = 0;
	percent_100_fr = 0;
	percent_10_fr = 0;
	percent_1_fr = 0;
	Checksum = 0;
}

bool Msg_F9::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 12) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_fr / (1.0/255)));
	bb.setOffset(3);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_ffr / (1.0/255)));
	bb.setOffset(4);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_cr / (1.0/255)));
	bb.setOffset(5);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_gr / (1.0/255)));
	bb.setOffset(6);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_100_fr / (1.0/255)));
	bb.setOffset(7);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_10_fr / (1.0/255)));
	bb.setOffset(8);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(percent_1_fr / (1.0/255)));
	int cIndex = 10;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 12);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_F9::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(2);	percent_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(3);	percent_ffr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(4);	percent_cr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(5);	percent_gr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(6);	percent_100_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(7);	percent_10_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(8);	percent_1_fr = static_cast<float>(bb.get<uint8_t>()) * (1.0/255);
	bb.setOffset(10);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 12);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

