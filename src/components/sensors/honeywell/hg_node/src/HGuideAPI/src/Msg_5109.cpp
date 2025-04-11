#include <include/HGuideAPI.h>
#include <include/Msg_5109.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5109::AddressId;
const uint32_t Msg_5109::MessageId;
const uint32_t Msg_5109::MessageLength;

Msg_5109::Msg_5109()
{
	Default();
}

void Msg_5109::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	Roll = 0;
	Pitch = 0;
	True_Heading = 0;
	Roll_Stdv = 0;
	Pitch_Stdv = 0;
	True_Heading_Stdv = 0;
	Data_source = 0;
	GPSMode = static_cast<gps_mode_table_t>(0);
}

bool Msg_5109::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 76) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(Roll);
	bb.setOffset(36);	bb.put(Pitch);
	bb.setOffset(40);	bb.put(True_Heading);
	bb.setOffset(48);	bb.put(Roll_Stdv);
	bb.setOffset(52);	bb.put(Pitch_Stdv);
	bb.setOffset(56);	bb.put(True_Heading_Stdv);
	bb.setOffset(60);	bb.put(Data_source);
	bb.setOffset(64);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(GPSMode));
	Checksum = computeChecksum((uint32_t*)buffer, 19);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5109::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 76) return -2;

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
	bb.setOffset(24);	bb.get(gpsTov);
	bb.setOffset(32);	bb.get(Roll);
	bb.setOffset(36);	bb.get(Pitch);
	bb.setOffset(40);	bb.get(True_Heading);
	bb.setOffset(48);	bb.get(Roll_Stdv);
	bb.setOffset(52);	bb.get(Pitch_Stdv);
	bb.setOffset(56);	bb.get(True_Heading_Stdv);
	bb.setOffset(60);	bb.get(Data_source);
	bb.setOffset(64);	GPSMode = static_cast<gps_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 19);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

