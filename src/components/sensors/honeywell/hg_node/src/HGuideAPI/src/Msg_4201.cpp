#include <include/HGuideAPI.h>
#include <include/Msg_4201.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4201::AddressId;
const uint32_t Msg_4201::MessageId;
const uint32_t Msg_4201::MessageLength;

Msg_4201::Msg_4201()
{
	Default();
}

void Msg_4201::Default()
{
	Checksum = 0;
	MarkPort = static_cast<event_in_mark_t>(0);
	Enable_6201_Time = 0;
	Enable_6202_Position = 0;
	Enable_6203_Velocity = 0;
	Enable_6204_Attitude = 0;
	Enable_6205_Full_State = 0;
	ChangeEventInSetUp = 0;
	Polarity = 0;
	time_offset = 0;
	time_guard = 0;
}

bool Msg_4201::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 28) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(MarkPort));

	uint8_t bitfieldAtByte18 = 0; // temporary variable holding the bitfield
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 0, Enable_6201_Time, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 1, Enable_6202_Position, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 2, Enable_6203_Velocity, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 3, Enable_6204_Attitude, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 4, Enable_6205_Full_State, status_ok);
	bitfieldAtByte18 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte18, 7, ChangeEventInSetUp, status_ok);
	bb.setOffset(18);	bb.put(bitfieldAtByte18);

	bb.setOffset(19);	bb.put(Polarity);
	bb.setOffset(20);	bb.put(time_offset);
	bb.setOffset(24);	bb.put(time_guard);
	Checksum = computeChecksum((uint32_t*)buffer, 7);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4201::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 28) return -2;

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
	bb.setOffset(16);	MarkPort = static_cast<event_in_mark_t>(bb.get<uint8_t>());

	uint8_t bitfieldAtByte18 = 0; // temporary variable holding the bitfield
	bb.setOffset(18);	bb.get(bitfieldAtByte18);
	Enable_6201_Time = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 0,  status_ok);
	Enable_6202_Position = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 1,  status_ok);
	Enable_6203_Velocity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 2,  status_ok);
	Enable_6204_Attitude = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 3,  status_ok);
	Enable_6205_Full_State = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 4,  status_ok);
	ChangeEventInSetUp = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte18, 7,  status_ok);
	bb.setOffset(19);	bb.get(Polarity);
	bb.setOffset(20);	bb.get(time_offset);
	bb.setOffset(24);	bb.get(time_guard);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 7);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

