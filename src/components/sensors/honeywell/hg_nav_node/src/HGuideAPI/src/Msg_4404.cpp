#include <include/HGuideAPI.h>
#include <include/Msg_4404.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4404::AddressId;
const uint32_t Msg_4404::MessageId;
const uint32_t Msg_4404::MessageLength;

Msg_4404::Msg_4404()
{
	Default();
}

void Msg_4404::Default()
{
	Checksum = 0;
	CasetoVehicleX = 0;
	CasetoVehicleY = 0;
	CasetoVehicleZ = 0;
	CasetoVehicleRoll = 0;
	CasetoVehiclePitch = 0;
	CasetoVehicleYaw = 0;
	ChangeVehicleFrameAttitude = 0;
	ChangeVehicleFrameOffset = 0;
}

bool Msg_4404::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 52) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(CasetoVehicleX);
	bb.setOffset(20);	bb.put(CasetoVehicleY);
	bb.setOffset(24);	bb.put(CasetoVehicleZ);
	bb.setOffset(28);	bb.put(CasetoVehicleRoll);
	bb.setOffset(32);	bb.put(CasetoVehiclePitch);
	bb.setOffset(36);	bb.put(CasetoVehicleYaw);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 4, ChangeVehicleFrameAttitude, status_ok);
	bitfieldAtByte48 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte48, 5, ChangeVehicleFrameOffset, status_ok);
	bb.setOffset(48);	bb.put(bitfieldAtByte48);

	Checksum = computeChecksum((uint32_t*)buffer, 13);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4404::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 52) return -2;

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
	bb.setOffset(16);	bb.get(CasetoVehicleX);
	bb.setOffset(20);	bb.get(CasetoVehicleY);
	bb.setOffset(24);	bb.get(CasetoVehicleZ);
	bb.setOffset(28);	bb.get(CasetoVehicleRoll);
	bb.setOffset(32);	bb.get(CasetoVehiclePitch);
	bb.setOffset(36);	bb.get(CasetoVehicleYaw);

	uint8_t bitfieldAtByte48 = 0; // temporary variable holding the bitfield
	bb.setOffset(48);	bb.get(bitfieldAtByte48);
	ChangeVehicleFrameAttitude = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 4,  status_ok);
	ChangeVehicleFrameOffset = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte48, 5,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 13);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

