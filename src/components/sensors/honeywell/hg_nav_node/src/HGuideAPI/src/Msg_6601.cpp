#include <include/HGuideAPI.h>
#include <include/Msg_6601.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6601::AddressId;
const uint32_t Msg_6601::MessageId;
const uint32_t Msg_6601::MessageLength;

Msg_6601::Msg_6601()
{
	Default();
}

void Msg_6601::Default()
{
	Checksum = 0;
	PWR_LED_State.Default();
	POS_LED_State.Default();
	SYS_LED_State.Default();
	LOG_LED_State.Default();
}

bool Msg_6601::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 32) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t PWR_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	PWR_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(PWR_LED_State_tmp, 0, 7, PWR_LED_State.blue_byte, status_ok);
	PWR_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(PWR_LED_State_tmp, 8, 15, PWR_LED_State.green_byte, status_ok);
	PWR_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(PWR_LED_State_tmp, 16, 23, PWR_LED_State.red_byte, status_ok);
	bb.setOffset(16);	bb.put(PWR_LED_State_tmp);


	uint32_t POS_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	POS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(POS_LED_State_tmp, 0, 7, POS_LED_State.blue_byte, status_ok);
	POS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(POS_LED_State_tmp, 8, 15, POS_LED_State.green_byte, status_ok);
	POS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(POS_LED_State_tmp, 16, 23, POS_LED_State.red_byte, status_ok);
	bb.setOffset(20);	bb.put(POS_LED_State_tmp);


	uint32_t SYS_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	SYS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(SYS_LED_State_tmp, 0, 7, SYS_LED_State.blue_byte, status_ok);
	SYS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(SYS_LED_State_tmp, 8, 15, SYS_LED_State.green_byte, status_ok);
	SYS_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(SYS_LED_State_tmp, 16, 23, SYS_LED_State.red_byte, status_ok);
	bb.setOffset(24);	bb.put(SYS_LED_State_tmp);


	uint32_t LOG_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	LOG_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(LOG_LED_State_tmp, 0, 7, LOG_LED_State.blue_byte, status_ok);
	LOG_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(LOG_LED_State_tmp, 8, 15, LOG_LED_State.green_byte, status_ok);
	LOG_LED_State_tmp = ECTOS::BIT_UTILITIES::Pack(LOG_LED_State_tmp, 16, 23, LOG_LED_State.red_byte, status_ok);
	bb.setOffset(28);	bb.put(LOG_LED_State_tmp);

	Checksum = computeChecksum((uint32_t*)buffer, 8);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6601::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 32) return -2;

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

	uint32_t PWR_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(PWR_LED_State_tmp);
	PWR_LED_State.blue_byte = ECTOS::BIT_UTILITIES::UnPack(PWR_LED_State_tmp, 0, 7,  status_ok);
	PWR_LED_State.green_byte = ECTOS::BIT_UTILITIES::UnPack(PWR_LED_State_tmp, 8, 15,  status_ok);
	PWR_LED_State.red_byte = ECTOS::BIT_UTILITIES::UnPack(PWR_LED_State_tmp, 16, 23,  status_ok);

	uint32_t POS_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(20);	bb.get(POS_LED_State_tmp);
	POS_LED_State.blue_byte = ECTOS::BIT_UTILITIES::UnPack(POS_LED_State_tmp, 0, 7,  status_ok);
	POS_LED_State.green_byte = ECTOS::BIT_UTILITIES::UnPack(POS_LED_State_tmp, 8, 15,  status_ok);
	POS_LED_State.red_byte = ECTOS::BIT_UTILITIES::UnPack(POS_LED_State_tmp, 16, 23,  status_ok);

	uint32_t SYS_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(24);	bb.get(SYS_LED_State_tmp);
	SYS_LED_State.blue_byte = ECTOS::BIT_UTILITIES::UnPack(SYS_LED_State_tmp, 0, 7,  status_ok);
	SYS_LED_State.green_byte = ECTOS::BIT_UTILITIES::UnPack(SYS_LED_State_tmp, 8, 15,  status_ok);
	SYS_LED_State.red_byte = ECTOS::BIT_UTILITIES::UnPack(SYS_LED_State_tmp, 16, 23,  status_ok);

	uint32_t LOG_LED_State_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(28);	bb.get(LOG_LED_State_tmp);
	LOG_LED_State.blue_byte = ECTOS::BIT_UTILITIES::UnPack(LOG_LED_State_tmp, 0, 7,  status_ok);
	LOG_LED_State.green_byte = ECTOS::BIT_UTILITIES::UnPack(LOG_LED_State_tmp, 8, 15,  status_ok);
	LOG_LED_State.red_byte = ECTOS::BIT_UTILITIES::UnPack(LOG_LED_State_tmp, 16, 23,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 8);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

