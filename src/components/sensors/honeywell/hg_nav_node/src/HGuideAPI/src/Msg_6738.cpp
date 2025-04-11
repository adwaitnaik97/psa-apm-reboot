#include <include/HGuideAPI.h>
#include <include/Msg_6738.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6738::AddressId;
const uint32_t Msg_6738::MessageId;
const uint32_t Msg_6738::MessageLength;

Msg_6738::Msg_6738()
{
	Default();
}

void Msg_6738::Default()
{
	Checksum = 0;
	systemTov = 0;
	velNormResBeam0BT = 0;
	velNormResBeam0WT = 0;
	velNormResBeam1BT = 0;
	velNormResBeam1WT = 0;
	velNormResBeam2BT = 0;
	velNormResBeam2WT = 0;
	velNormResBeam3BT = 0;
	velNormResBeam3WT = 0;
	sfErrorEstBeam0 = 0;
	sfErrorEstBeam1 = 0;
	sfErrorEstBeam2 = 0;
	sfErrorEstBeam3 = 0;
	psiYErrorEstBeam0 = 0;
	psiYErrorEstBeam1 = 0;
	psiYErrorEstBeam2 = 0;
	psiYErrorEstBeam3 = 0;
	psiZErrorEstBeam0 = 0;
	psiZErrorEstBeam1 = 0;
	psiZErrorEstBeam2 = 0;
	psiZErrorEstBeam3 = 0;
	oceanCurErrorEstBeam0 = 0;
	oceanCurErrorEstBeam1 = 0;
	oceanCurErrorEstBeam2 = 0;
	oceanCurErrorEstBeam3 = 0;
	sfStdvBeam0 = 0;
	sfStdvBeam1 = 0;
	sfStdvBeam2 = 0;
	sfStdvBeam3 = 0;
	psiYStdvBeam0 = 0;
	psiYStdvBeam1 = 0;
	psiYStdvBeam2 = 0;
	psiYStdvBeam3 = 0;
	psiZStdvBeam0 = 0;
	psiZStdvBeam1 = 0;
	psiZStdvBeam2 = 0;
	psiZStdvBeam3 = 0;
	oceanCurStdvBeam0 = 0;
	oceanCurStdvBeam1 = 0;
	oceanCurStdvBeam2 = 0;
	oceanCurStdvBeam3 = 0;
}

bool Msg_6738::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 192) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(velNormResBeam0BT);
	bb.setOffset(36);	bb.put(velNormResBeam0WT);
	bb.setOffset(40);	bb.put(velNormResBeam1BT);
	bb.setOffset(44);	bb.put(velNormResBeam1WT);
	bb.setOffset(48);	bb.put(velNormResBeam2BT);
	bb.setOffset(52);	bb.put(velNormResBeam2WT);
	bb.setOffset(56);	bb.put(velNormResBeam3BT);
	bb.setOffset(60);	bb.put(velNormResBeam3WT);
	bb.setOffset(64);	bb.put(sfErrorEstBeam0);
	bb.setOffset(68);	bb.put(sfErrorEstBeam1);
	bb.setOffset(72);	bb.put(sfErrorEstBeam2);
	bb.setOffset(76);	bb.put(sfErrorEstBeam3);
	bb.setOffset(80);	bb.put(psiYErrorEstBeam0);
	bb.setOffset(84);	bb.put(psiYErrorEstBeam1);
	bb.setOffset(88);	bb.put(psiYErrorEstBeam2);
	bb.setOffset(92);	bb.put(psiYErrorEstBeam3);
	bb.setOffset(96);	bb.put(psiZErrorEstBeam0);
	bb.setOffset(100);	bb.put(psiZErrorEstBeam1);
	bb.setOffset(104);	bb.put(psiZErrorEstBeam2);
	bb.setOffset(108);	bb.put(psiZErrorEstBeam3);
	bb.setOffset(112);	bb.put(oceanCurErrorEstBeam0);
	bb.setOffset(116);	bb.put(oceanCurErrorEstBeam1);
	bb.setOffset(120);	bb.put(oceanCurErrorEstBeam2);
	bb.setOffset(124);	bb.put(oceanCurErrorEstBeam3);
	bb.setOffset(128);	bb.put(sfStdvBeam0);
	bb.setOffset(132);	bb.put(sfStdvBeam1);
	bb.setOffset(136);	bb.put(sfStdvBeam2);
	bb.setOffset(140);	bb.put(sfStdvBeam3);
	bb.setOffset(144);	bb.put(psiYStdvBeam0);
	bb.setOffset(148);	bb.put(psiYStdvBeam1);
	bb.setOffset(152);	bb.put(psiYStdvBeam2);
	bb.setOffset(156);	bb.put(psiYStdvBeam3);
	bb.setOffset(160);	bb.put(psiZStdvBeam0);
	bb.setOffset(164);	bb.put(psiZStdvBeam1);
	bb.setOffset(168);	bb.put(psiZStdvBeam2);
	bb.setOffset(172);	bb.put(psiZStdvBeam3);
	bb.setOffset(176);	bb.put(oceanCurStdvBeam0);
	bb.setOffset(180);	bb.put(oceanCurStdvBeam1);
	bb.setOffset(184);	bb.put(oceanCurStdvBeam2);
	bb.setOffset(188);	bb.put(oceanCurStdvBeam3);
	Checksum = computeChecksum((uint32_t*)buffer, 48);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6738::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 192) return -2;

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
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(velNormResBeam0BT);
	bb.setOffset(36);	bb.get(velNormResBeam0WT);
	bb.setOffset(40);	bb.get(velNormResBeam1BT);
	bb.setOffset(44);	bb.get(velNormResBeam1WT);
	bb.setOffset(48);	bb.get(velNormResBeam2BT);
	bb.setOffset(52);	bb.get(velNormResBeam2WT);
	bb.setOffset(56);	bb.get(velNormResBeam3BT);
	bb.setOffset(60);	bb.get(velNormResBeam3WT);
	bb.setOffset(64);	bb.get(sfErrorEstBeam0);
	bb.setOffset(68);	bb.get(sfErrorEstBeam1);
	bb.setOffset(72);	bb.get(sfErrorEstBeam2);
	bb.setOffset(76);	bb.get(sfErrorEstBeam3);
	bb.setOffset(80);	bb.get(psiYErrorEstBeam0);
	bb.setOffset(84);	bb.get(psiYErrorEstBeam1);
	bb.setOffset(88);	bb.get(psiYErrorEstBeam2);
	bb.setOffset(92);	bb.get(psiYErrorEstBeam3);
	bb.setOffset(96);	bb.get(psiZErrorEstBeam0);
	bb.setOffset(100);	bb.get(psiZErrorEstBeam1);
	bb.setOffset(104);	bb.get(psiZErrorEstBeam2);
	bb.setOffset(108);	bb.get(psiZErrorEstBeam3);
	bb.setOffset(112);	bb.get(oceanCurErrorEstBeam0);
	bb.setOffset(116);	bb.get(oceanCurErrorEstBeam1);
	bb.setOffset(120);	bb.get(oceanCurErrorEstBeam2);
	bb.setOffset(124);	bb.get(oceanCurErrorEstBeam3);
	bb.setOffset(128);	bb.get(sfStdvBeam0);
	bb.setOffset(132);	bb.get(sfStdvBeam1);
	bb.setOffset(136);	bb.get(sfStdvBeam2);
	bb.setOffset(140);	bb.get(sfStdvBeam3);
	bb.setOffset(144);	bb.get(psiYStdvBeam0);
	bb.setOffset(148);	bb.get(psiYStdvBeam1);
	bb.setOffset(152);	bb.get(psiYStdvBeam2);
	bb.setOffset(156);	bb.get(psiYStdvBeam3);
	bb.setOffset(160);	bb.get(psiZStdvBeam0);
	bb.setOffset(164);	bb.get(psiZStdvBeam1);
	bb.setOffset(168);	bb.get(psiZStdvBeam2);
	bb.setOffset(172);	bb.get(psiZStdvBeam3);
	bb.setOffset(176);	bb.get(oceanCurStdvBeam0);
	bb.setOffset(180);	bb.get(oceanCurStdvBeam1);
	bb.setOffset(184);	bb.get(oceanCurStdvBeam2);
	bb.setOffset(188);	bb.get(oceanCurStdvBeam3);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 48);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

