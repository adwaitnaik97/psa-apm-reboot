#include <include/HGuideAPI.h>
#include <include/Msg_5201.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_5201::AddressId;
const uint32_t Msg_5201::MessageId;
const uint32_t Msg_5201::MessageLength;

Msg_5201::Msg_5201()
{
	Default();
}

void Msg_5201::Default()
{
	Checksum = 0;
	gpsTovValid = 0;
	gpsToUTCofsetValid = 0;
	gps_time_to_utc_offset = 0;
	utc_time_figure_of_merit = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	utc_day_of_week = 0;
	utc_day_of_year = 0;
	utc_day_of_month = 0;
	utc_month = 0;
	utc_year = 0;
	utc_second = 0;
	utc_minute = 0;
	utc_hour = 0;
}

bool Msg_5201::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 72) return false;

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
	bb.setOffset(28);	bb.put(utc_time_figure_of_merit);
	bb.setOffset(32);	bb.put(systemTov);
	bb.setOffset(40);	bb.put(gpsTov);
	bb.setOffset(48);	bb.put(gps_week);

	uint32_t bitfieldAtByte52 = 0; // temporary variable holding the bitfield
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte52, 0, 2, utc_day_of_week, status_ok);
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte52, 3, 11, utc_day_of_year, status_ok);
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte52, 12, 16, utc_day_of_month, status_ok);
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte52, 17, 20, utc_month, status_ok);
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte52, 21, 31, utc_year, status_ok);
	bb.setOffset(52);	bb.put(bitfieldAtByte52);


	uint8_t bitfieldAtByte56 = 0; // temporary variable holding the bitfield
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte56, 14, 19, utc_second, status_ok);
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte56, 20, 25, utc_minute, status_ok);
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte56, 26, 31, utc_hour, status_ok);
	bb.setOffset(56);	bb.put(bitfieldAtByte56);

	Checksum = computeChecksum((uint32_t*)buffer, 18);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_5201::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 72) return -2;

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
	bb.setOffset(28);	bb.get(utc_time_figure_of_merit);
	bb.setOffset(32);	bb.get(systemTov);
	bb.setOffset(40);	bb.get(gpsTov);
	bb.setOffset(48);	bb.get(gps_week);

	uint32_t bitfieldAtByte52 = 0; // temporary variable holding the bitfield
	bb.setOffset(52);	bb.get(bitfieldAtByte52);
	utc_day_of_week = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte52, 0, 2,  status_ok);
	utc_day_of_year = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte52, 3, 11,  status_ok);
	utc_day_of_month = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte52, 12, 16,  status_ok);
	utc_month = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte52, 17, 20,  status_ok);
	utc_year = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte52, 21, 31,  status_ok);

	uint8_t bitfieldAtByte56 = 0; // temporary variable holding the bitfield
	bb.setOffset(56);	bb.get(bitfieldAtByte56);
	utc_second = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte56, 14, 19,  status_ok);
	utc_minute = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte56, 20, 25,  status_ok);
	utc_hour = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte56, 26, 31,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 18);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

