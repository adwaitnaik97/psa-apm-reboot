#include <include/HGuideAPI.h>
#include <include/Msg_4738.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4738::AddressId;
const uint32_t Msg_4738::MessageId;
const uint32_t Msg_4738::MessageLength;

Msg_4738::Msg_4738()
{
	Default();
}

void Msg_4738::Default()
{
	Checksum = 0;
	measNoise = 0;
	measRejectThreshold = 0;
	sfUncertainty = 0;
	sfTimeConst = 0;
	misalignInitUncertainty = 0;
	misalignTimeConst = 0;
	pitchBoresightInitUncertainty = 0;
	pitchBoresightStateNoise = 0;
	for (unsigned int index = 0; index < 3; index++)
	{
		leverarmBeam0[index] = 0;
	}
	psiRotationBeam0 = 0;
	for (unsigned int index = 0; index < 3; index++)
	{
		leverarmBeam1[index] = 0;
	}
	psiRotationBeam1 = 0;
	for (unsigned int index = 0; index < 3; index++)
	{
		leverarmBeam2[index] = 0;
	}
	psiRotationBeam2 = 0;
	for (unsigned int index = 0; index < 3; index++)
	{
		leverarmBeam3[index] = 0;
	}
	psiRotationBeam3 = 0;
	psiThtRotation = 0;
	psiOffset = 0;
	saveCfgToFlash = 0;
}

bool Msg_4738::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 144) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(measNoise);
	bb.setOffset(20);	bb.put(measRejectThreshold);
	bb.setOffset(24);	bb.put(sfUncertainty);
	bb.setOffset(28);	bb.put(sfTimeConst);
	bb.setOffset(32);	bb.put(misalignInitUncertainty);
	bb.setOffset(36);	bb.put(misalignTimeConst);
	bb.setOffset(40);	bb.put(pitchBoresightInitUncertainty);
	bb.setOffset(44);	bb.put(pitchBoresightStateNoise);
	bb.setOffset(48);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.put(leverarmBeam0[index]);
	}
	bb.setOffset(60);	bb.put(psiRotationBeam0);
	bb.setOffset(64);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.put(leverarmBeam1[index]);
	}
	bb.setOffset(76);	bb.put(psiRotationBeam1);
	bb.setOffset(80);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.put(leverarmBeam2[index]);
	}
	bb.setOffset(92);	bb.put(psiRotationBeam2);
	bb.setOffset(96);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.put(leverarmBeam3[index]);
	}
	bb.setOffset(108);	bb.put(psiRotationBeam3);
	bb.setOffset(112);	bb.put(psiThtRotation);
	bb.setOffset(116);	bb.put(psiOffset);
	bb.setOffset(140);	bb.put(saveCfgToFlash);
	Checksum = computeChecksum((uint32_t*)buffer, 36);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4738::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 144) return -2;

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
	bb.setOffset(16);	bb.get(measNoise);
	bb.setOffset(20);	bb.get(measRejectThreshold);
	bb.setOffset(24);	bb.get(sfUncertainty);
	bb.setOffset(28);	bb.get(sfTimeConst);
	bb.setOffset(32);	bb.get(misalignInitUncertainty);
	bb.setOffset(36);	bb.get(misalignTimeConst);
	bb.setOffset(40);	bb.get(pitchBoresightInitUncertainty);
	bb.setOffset(44);	bb.get(pitchBoresightStateNoise);
	bb.setOffset(48);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.get(leverarmBeam0[index]);
	}
	bb.setOffset(60);	bb.get(psiRotationBeam0);
	bb.setOffset(64);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.get(leverarmBeam1[index]);
	}
	bb.setOffset(76);	bb.get(psiRotationBeam1);
	bb.setOffset(80);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.get(leverarmBeam2[index]);
	}
	bb.setOffset(92);	bb.get(psiRotationBeam2);
	bb.setOffset(96);
	for (unsigned int index = 0; index < 3; index++)
	{
		bb.get(leverarmBeam3[index]);
	}
	bb.setOffset(108);	bb.get(psiRotationBeam3);
	bb.setOffset(112);	bb.get(psiThtRotation);
	bb.setOffset(116);	bb.get(psiOffset);
	bb.setOffset(140);	bb.get(saveCfgToFlash);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 36);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

