#include <include/HGuideAPI.h>
#include <include/Msg_6211.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6211::AddressId;
const uint32_t Msg_6211::MessageId;
const uint32_t Msg_6211::MessageLength;

Msg_6211::Msg_6211()
{
	Default();
}

void Msg_6211::Default()
{
	Checksum = 0;
	markPort = static_cast<event_out_mark_t>(0);
	system_time_of_event_in = 0;
	gps_time_of_event_in = 0;
	gps_week_of_event_in = 0;
}

bool Msg_6211::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 40) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(markPort));
	bb.setOffset(20);	bb.put(system_time_of_event_in);
	bb.setOffset(28);	bb.put(gps_time_of_event_in);
	bb.setOffset(36);	bb.put(gps_week_of_event_in);
	Checksum = computeChecksum((uint32_t*)buffer, 10);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6211::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 40) return -2;

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
	bb.setOffset(16);	markPort = static_cast<event_out_mark_t>(bb.get<uint32_t>());
	bb.setOffset(20);	bb.get(system_time_of_event_in);
	bb.setOffset(28);	bb.get(gps_time_of_event_in);
	bb.setOffset(36);	bb.get(gps_week_of_event_in);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 10);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

