#include <include/HGuideAPI.h>
#include <include/Msg_B4.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_B4::SyncByte;
const uint8_t Msg_B4::MessageID;

Msg_B4::Msg_B4()
{
	Default();
}

void Msg_B4::Default()
{
	session = 0;
	systemTov = 0;
	attitude_i = 0;
	attitude_j = 0;
	attitude_k = 0;
	attitude_s = 0;
	temperature = 0;
	status_word.Default();
	Checksum = 0;
}

bool Msg_B4::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 26) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(session);
	bb.setOffset(4);	bb.put(systemTov);
	bb.setOffset(12);	bb.put(attitude_i);
	bb.setOffset(14);	bb.put(attitude_j);
	bb.setOffset(16);	bb.put(attitude_k);
	bb.setOffset(18);	bb.put(attitude_s);
	bb.setOffset(20);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(temperature / (std::pow(2, -8))));

	uint16_t status_word_tmp = 0; // temporary variable holding the custom bitfield
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 0, status_word.gyro_x_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 1, status_word.gyro_y_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 2, status_word.gyro_z_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 3, status_word.accel_x_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 4, status_word.accel_y_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 5, status_word.accel_z_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 6, status_word.mag_x_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 7, status_word.mag_y_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::PackBool(status_word_tmp, 8, status_word.mag_z_fail, status_ok);
	status_word_tmp = ECTOS::BIT_UTILITIES::Pack(status_word_tmp, 12, 15, status_word.counter, status_ok);
	bb.setOffset(22);	bb.put(status_word_tmp);

	int cIndex = 24;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 26);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_B4::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 26) return -2;

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
	bb.setOffset(2);	bb.get(session);
	bb.setOffset(4);	bb.get(systemTov);
	bb.setOffset(12);	bb.get(attitude_i);
	bb.setOffset(14);	bb.get(attitude_j);
	bb.setOffset(16);	bb.get(attitude_k);
	bb.setOffset(18);	bb.get(attitude_s);
	bb.setOffset(20);	temperature = static_cast<float>(bb.get<int16_t>()) * (std::pow(2, -8));

	uint16_t status_word_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(22);	bb.get(status_word_tmp);
	status_word.gyro_x_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 0,  status_ok);
	status_word.gyro_y_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 1,  status_ok);
	status_word.gyro_z_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 2,  status_ok);
	status_word.accel_x_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 3,  status_ok);
	status_word.accel_y_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 4,  status_ok);
	status_word.accel_z_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 5,  status_ok);
	status_word.mag_x_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 6,  status_ok);
	status_word.mag_y_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 7,  status_ok);
	status_word.mag_z_fail = ECTOS::BIT_UTILITIES::UnPackBool(status_word_tmp, 8,  status_ok);
	status_word.counter = ECTOS::BIT_UTILITIES::UnPack(status_word_tmp, 12, 15,  status_ok);
	bb.setOffset(24);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 26);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

