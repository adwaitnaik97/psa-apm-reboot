#include <include/HGuideAPI.h>
#include <include/Msg_1201.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1201::AddressId;
const uint32_t Msg_1201::MessageId;
const uint32_t Msg_1201::MessageLength;

Msg_1201::Msg_1201()
{
	Default();
}

void Msg_1201::Default()
{
	Checksum = 0;
	gpsTovValid = 0;
	gpsToUTCofsetValid = 0;
	gps_time_to_utc_offset = 0;
	PPSgpsTov = 0;
	gps_week = 0;
}

bool Msg_1201::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 60) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint8_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 0, gpsTovValid, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 3, gpsToUTCofsetValid, status_ok);
	bb.setOffset(16);	bb.put(bitfieldAtByte16);

	bb.setOffset(20);	bb.put(gps_time_to_utc_offset);
	bb.setOffset(32);	bb.put(PPSgpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	Checksum = computeChecksum((uint32_t*)buffer, 15);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1201::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 60) return -2;

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

	uint8_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bb.setOffset(16);	bb.get(bitfieldAtByte16);
	gpsTovValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 0,  status_ok);
	gpsToUTCofsetValid = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 3,  status_ok);
	bb.setOffset(20);	bb.get(gps_time_to_utc_offset);
	bb.setOffset(32);	bb.get(PPSgpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 15);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

