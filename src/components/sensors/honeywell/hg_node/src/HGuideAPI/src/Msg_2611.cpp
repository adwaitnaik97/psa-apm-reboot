#include <include/HGuideAPI.h>
#include <include/Msg_2611.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2611::AddressId;
const uint32_t Msg_2611::MessageId;
const uint32_t Msg_2611::MessageLength;

Msg_2611::Msg_2611()
{
	Default();
}

void Msg_2611::Default()
{
	Checksum = 0;
	Last_Message = 0;
	messageNumber_a = 0;
	faultID_a = 0;
	BITmode_a = 0;
	systemMode_a = 0;
	powerCycleCount_a = 0;
	faultValue_a = 0;
	channelSubsystem_a = 0;
	eti_a = 0;
	angularRateX_a = 0;
	angularRateY_a = 0;
	angularRateZ_a = 0;
	accelerationX_a = 0;
	accelerationY_a = 0;
	accelerationZ_a = 0;
	velocityX_a = 0;
	velocityY_a = 0;
	velocityZ_a = 0;
	insTemperature_a = 0;
	altitude_a = 0;
	messageNumber_b = 0;
	faultID_b = 0;
	BITmode_b = 0;
	systemMode_b = 0;
	powerCycleCount_b = 0;
	faultValue_b = 0;
	channelSubsystem_b = 0;
	eti_b = 0;
	angularRateX_b = 0;
	angularRateY_b = 0;
	angularRateZ_b = 0;
	accelerationX_b = 0;
	accelerationY_b = 0;
	accelerationZ_b = 0;
	velocityX_b = 0;
	velocityY_b = 0;
	velocityZ_b = 0;
	insTemperature_b = 0;
	altitude_b = 0;
}

bool Msg_2611::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 228) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(Last_Message);
	bb.setOffset(28);	bb.put(messageNumber_a);
	bb.setOffset(32);	bb.put(faultID_a);
	bb.setOffset(36);	bb.put(BITmode_a);
	bb.setOffset(40);	bb.put(systemMode_a);
	bb.setOffset(44);	bb.put(powerCycleCount_a);
	bb.setOffset(48);	bb.put(faultValue_a);
	bb.setOffset(52);	bb.put(channelSubsystem_a);
	bb.setOffset(56);	bb.put(eti_a);
	bb.setOffset(60);	bb.put(angularRateX_a);
	bb.setOffset(64);	bb.put(angularRateY_a);
	bb.setOffset(68);	bb.put(angularRateZ_a);
	bb.setOffset(72);	bb.put(accelerationX_a);
	bb.setOffset(76);	bb.put(accelerationY_a);
	bb.setOffset(80);	bb.put(accelerationZ_a);
	bb.setOffset(84);	bb.put(velocityX_a);
	bb.setOffset(88);	bb.put(velocityY_a);
	bb.setOffset(92);	bb.put(velocityZ_a);
	bb.setOffset(96);	bb.put(insTemperature_a);
	bb.setOffset(100);	bb.put(altitude_a);
	bb.setOffset(128);	bb.put(messageNumber_b);
	bb.setOffset(132);	bb.put(faultID_b);
	bb.setOffset(136);	bb.put(BITmode_b);
	bb.setOffset(140);	bb.put(systemMode_b);
	bb.setOffset(144);	bb.put(powerCycleCount_b);
	bb.setOffset(148);	bb.put(faultValue_b);
	bb.setOffset(152);	bb.put(channelSubsystem_b);
	bb.setOffset(156);	bb.put(eti_b);
	bb.setOffset(160);	bb.put(angularRateX_b);
	bb.setOffset(164);	bb.put(angularRateY_b);
	bb.setOffset(168);	bb.put(angularRateZ_b);
	bb.setOffset(172);	bb.put(accelerationX_b);
	bb.setOffset(176);	bb.put(accelerationY_b);
	bb.setOffset(180);	bb.put(accelerationZ_b);
	bb.setOffset(184);	bb.put(velocityX_b);
	bb.setOffset(188);	bb.put(velocityY_b);
	bb.setOffset(192);	bb.put(velocityZ_b);
	bb.setOffset(196);	bb.put(insTemperature_b);
	bb.setOffset(200);	bb.put(altitude_b);
	Checksum = computeChecksum((uint32_t*)buffer, 57);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2611::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 228) return -2;

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
	bb.setOffset(16);	bb.get(Last_Message);
	bb.setOffset(28);	bb.get(messageNumber_a);
	bb.setOffset(32);	bb.get(faultID_a);
	bb.setOffset(36);	bb.get(BITmode_a);
	bb.setOffset(40);	bb.get(systemMode_a);
	bb.setOffset(44);	bb.get(powerCycleCount_a);
	bb.setOffset(48);	bb.get(faultValue_a);
	bb.setOffset(52);	bb.get(channelSubsystem_a);
	bb.setOffset(56);	bb.get(eti_a);
	bb.setOffset(60);	bb.get(angularRateX_a);
	bb.setOffset(64);	bb.get(angularRateY_a);
	bb.setOffset(68);	bb.get(angularRateZ_a);
	bb.setOffset(72);	bb.get(accelerationX_a);
	bb.setOffset(76);	bb.get(accelerationY_a);
	bb.setOffset(80);	bb.get(accelerationZ_a);
	bb.setOffset(84);	bb.get(velocityX_a);
	bb.setOffset(88);	bb.get(velocityY_a);
	bb.setOffset(92);	bb.get(velocityZ_a);
	bb.setOffset(96);	bb.get(insTemperature_a);
	bb.setOffset(100);	bb.get(altitude_a);
	bb.setOffset(128);	bb.get(messageNumber_b);
	bb.setOffset(132);	bb.get(faultID_b);
	bb.setOffset(136);	bb.get(BITmode_b);
	bb.setOffset(140);	bb.get(systemMode_b);
	bb.setOffset(144);	bb.get(powerCycleCount_b);
	bb.setOffset(148);	bb.get(faultValue_b);
	bb.setOffset(152);	bb.get(channelSubsystem_b);
	bb.setOffset(156);	bb.get(eti_b);
	bb.setOffset(160);	bb.get(angularRateX_b);
	bb.setOffset(164);	bb.get(angularRateY_b);
	bb.setOffset(168);	bb.get(angularRateZ_b);
	bb.setOffset(172);	bb.get(accelerationX_b);
	bb.setOffset(176);	bb.get(accelerationY_b);
	bb.setOffset(180);	bb.get(accelerationZ_b);
	bb.setOffset(184);	bb.get(velocityX_b);
	bb.setOffset(188);	bb.get(velocityY_b);
	bb.setOffset(192);	bb.get(velocityZ_b);
	bb.setOffset(196);	bb.get(insTemperature_b);
	bb.setOffset(200);	bb.get(altitude_b);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 57);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

