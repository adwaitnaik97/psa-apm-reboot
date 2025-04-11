#include <include/HGuideAPI.h>
#include <include/Msg_A9.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_A9::SyncByte;
const uint8_t Msg_A9::MessageID;

Msg_A9::Msg_A9()
{
	Default();
}

void Msg_A9::Default()
{
	DeltaAngleX = 0;
	DeltaAngleY = 0;
	DeltaAngleZ = 0;
	DeltaVelocityX = 0;
	DeltaVelocityY = 0;
	DeltaVelocityZ = 0;
	mg1FluxA = 0;
	mg1FluxB = 0;
	mg1FluxC = 0;
	mg1Temperature = 0;
	mg2FluxA = 0;
	mg2FluxB = 0;
	mg2FluxC = 0;
	mg2Temperature = 0;
	mg3FluxA = 0;
	mg3FluxB = 0;
	mg3FluxC = 0;
	mg3Temperature = 0;
	Checksum = 0;
}

bool Msg_A9::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 56) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleX / (1.164153E-10)));
	bb.setOffset(6);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleY / (1.164153E-10)));
	bb.setOffset(10);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleZ / (1.164153E-10)));
	bb.setOffset(14);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityX / (7.450581E-9)));
	bb.setOffset(18);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityY / (7.450581E-9)));
	bb.setOffset(22);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityZ / (7.450581E-9)));
	bb.setOffset(26);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg1FluxA / (0.000438404)));
	bb.setOffset(28);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg1FluxB / (0.000438404)));
	bb.setOffset(30);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg1FluxC / (0.000438404)));
	bb.setOffset(32);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg1Temperature / (0.003906250)));
	bb.setOffset(34);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg2FluxA / (0.000438404)));
	bb.setOffset(36);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg2FluxB / (0.000438404)));
	bb.setOffset(38);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg2FluxC / (0.000438404)));
	bb.setOffset(40);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg2Temperature / (0.003906250)));
	bb.setOffset(42);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg3FluxA / (0.000438404)));
	bb.setOffset(44);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg3FluxB / (0.000438404)));
	bb.setOffset(46);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg3FluxC / (0.000438404)));
	bb.setOffset(48);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(mg3Temperature / (0.003906250)));
	int cIndex = 54;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 56);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_A9::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 56) return -2;

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
	bb.setOffset(2);	DeltaAngleX = static_cast<float>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(6);	DeltaAngleY = static_cast<float>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(10);	DeltaAngleZ = static_cast<float>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(14);	DeltaVelocityX = static_cast<float>(bb.get<int32_t>()) * (7.450581E-9);
	bb.setOffset(18);	DeltaVelocityY = static_cast<float>(bb.get<int32_t>()) * (7.450581E-9);
	bb.setOffset(22);	DeltaVelocityZ = static_cast<float>(bb.get<int32_t>()) * (7.450581E-9);
	bb.setOffset(26);	mg1FluxA = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(28);	mg1FluxB = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(30);	mg1FluxC = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(32);	mg1Temperature = static_cast<float>(bb.get<int16_t>()) * (0.003906250);
	bb.setOffset(34);	mg2FluxA = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(36);	mg2FluxB = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(38);	mg2FluxC = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(40);	mg2Temperature = static_cast<float>(bb.get<int16_t>()) * (0.003906250);
	bb.setOffset(42);	mg3FluxA = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(44);	mg3FluxB = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(46);	mg3FluxC = static_cast<float>(bb.get<int16_t>()) * (0.000438404);
	bb.setOffset(48);	mg3Temperature = static_cast<float>(bb.get<int16_t>()) * (0.003906250);
	bb.setOffset(54);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 56);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

