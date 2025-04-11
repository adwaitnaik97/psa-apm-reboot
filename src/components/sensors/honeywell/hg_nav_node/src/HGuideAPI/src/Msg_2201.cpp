#include <include/HGuideAPI.h>
#include <include/Msg_2201.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2201::AddressId;
const uint32_t Msg_2201::MessageId;
const uint32_t Msg_2201::MessageLength;

Msg_2201::Msg_2201()
{
	Default();
}

void Msg_2201::Default()
{
	Checksum = 0;
	time_validity_bits.Default();
	utc_time_figure_of_merit = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	utc_day_month_year_date = 0;
	utc_day_of_week = 0;
	utc_day_of_year = 0;
	utc_day_of_month = 0;
	utc_month = 0;
	utc_year = 0;
	utc_hour_min_sec = 0;
	utc_second = 0;
	utc_minute = 0;
	utc_hour = 0;
	utc_time = 0;
	ins_gps_internal_system_time_pps_in = 0;
	gps_time_pps_in = 0;
	pps_in_count = 0;
}

bool Msg_2201::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 96) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t time_validity_bits_tmp = 0; // temporary variable holding the custom bitfield
	time_validity_bits_tmp = ECTOS::BIT_UTILITIES::PackBool(time_validity_bits_tmp, 0, time_validity_bits.gpsTovValid, status_ok);
	time_validity_bits_tmp = ECTOS::BIT_UTILITIES::PackBool(time_validity_bits_tmp, 1, time_validity_bits.utcTimeValid, status_ok);
	time_validity_bits_tmp = ECTOS::BIT_UTILITIES::Pack(time_validity_bits_tmp, 4, 5, time_validity_bits.PPS_Mode, status_ok);
	bb.setOffset(16);	bb.put(time_validity_bits_tmp);

	bb.setOffset(20);	bb.put(utc_time_figure_of_merit);
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(utc_day_month_year_date);

	uint32_t bitfieldAtByte44 = 0; // temporary variable holding the bitfield
	bitfieldAtByte44 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte44, 0, 2, utc_day_of_week, status_ok);
	bitfieldAtByte44 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte44, 3, 11, utc_day_of_year, status_ok);
	bitfieldAtByte44 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte44, 12, 16, utc_day_of_month, status_ok);
	bitfieldAtByte44 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte44, 17, 20, utc_month, status_ok);
	bitfieldAtByte44 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte44, 21, 31, utc_year, status_ok);
	bb.setOffset(44);	bb.put(bitfieldAtByte44);

	bb.setOffset(48);	bb.put(utc_hour_min_sec);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte48, 14, 19, utc_second, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte48, 20, 25, utc_minute, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte48, 26, 31, utc_hour, status_ok);
	bb.setOffset(48);	bb.put(bitfieldAtByte48);

	bb.setOffset(52);	bb.put(utc_time);
	bb.setOffset(60);	bb.put(ins_gps_internal_system_time_pps_in);
	bb.setOffset(68);	bb.put(gps_time_pps_in);
	bb.setOffset(76);	bb.put(pps_in_count);
	Checksum = computeChecksum((uint32_t*)buffer, 24);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2201::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 96) return -2;

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

	uint32_t time_validity_bits_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(time_validity_bits_tmp);
	time_validity_bits.gpsTovValid = ECTOS::BIT_UTILITIES::UnPackBool(time_validity_bits_tmp, 0,  status_ok);
	time_validity_bits.utcTimeValid = ECTOS::BIT_UTILITIES::UnPackBool(time_validity_bits_tmp, 1,  status_ok);
	time_validity_bits.PPS_Mode = ECTOS::BIT_UTILITIES::UnPack(time_validity_bits_tmp, 4, 5,  status_ok);
	bb.setOffset(20);	bb.get(utc_time_figure_of_merit);
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(utc_day_month_year_date);

	uint32_t bitfieldAtByte44 = 0; // temporary variable holding the bitfield
	bb.setOffset(44);	bb.get(bitfieldAtByte44);
	utc_day_of_week = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte44, 0, 2,  status_ok);
	utc_day_of_year = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte44, 3, 11,  status_ok);
	utc_day_of_month = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte44, 12, 16,  status_ok);
	utc_month = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte44, 17, 20,  status_ok);
	utc_year = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte44, 21, 31,  status_ok);
	bb.setOffset(48);	bb.get(utc_hour_min_sec);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bb.setOffset(48);	bb.get(bitfieldAtByte48);
	utc_second = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte48, 14, 19,  status_ok);
	utc_minute = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte48, 20, 25,  status_ok);
	utc_hour = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte48, 26, 31,  status_ok);
	bb.setOffset(52);	bb.get(utc_time);
	bb.setOffset(60);	bb.get(ins_gps_internal_system_time_pps_in);
	bb.setOffset(68);	bb.get(gps_time_pps_in);
	bb.setOffset(76);	bb.get(pps_in_count);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 24);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

