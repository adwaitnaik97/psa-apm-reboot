#include <include/HGuideAPI.h>
#include <include/Msg_2425.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2425::AddressId;
const uint32_t Msg_2425::MessageId;
const uint32_t Msg_2425::MessageLength;

Msg_2425::Msg_2425()
{
	Default();
}

void Msg_2425::Default()
{
	Checksum = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	TA_PosAidingX_ErrEst = 0;
	TA_PosAidingY_ErrEst = 0;
	TA_PosAidingZ_ErrEst = 0;
	TA_VelAidingX_ErrEst = 0;
	TA_VelAidingY_ErrEst = 0;
	TA_VelAidingZ_ErrEst = 0;
	TA_LeverArmX_ErrEst = 0;
	TA_LeverArmY_ErrEst = 0;
	TA_LeverArmZ_ErrEst = 0;
	TA_BoresightX_ErrEst = 0;
	TA_BoresightY_ErrEst = 0;
	TA_BoresightZ_ErrEst = 0;
	TA_PosAidingX_Stdv = 0;
	TA_PosAidingY_Stdv = 0;
	TA_PosAidingZ_Stdv = 0;
	TA_VelAidingX_Stdv = 0;
	TA_VelAidingY_Stdv = 0;
	TA_VelAidingZ_Stdv = 0;
	TA_LeverArmX_Stdv = 0;
	TA_LeverArmY_Stdv = 0;
	TA_LeverArmZ_Stdv = 0;
	TA_BoresightX_Stdv = 0;
	TA_BoresightY_Stdv = 0;
	TA_BoresightZ_Stdv = 0;
}

bool Msg_2425::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 164) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INSMode));
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(TA_PosAidingX_ErrEst);
	bb.setOffset(48);	bb.put(TA_PosAidingY_ErrEst);
	bb.setOffset(52);	bb.put(TA_PosAidingZ_ErrEst);
	bb.setOffset(56);	bb.put(TA_VelAidingX_ErrEst);
	bb.setOffset(60);	bb.put(TA_VelAidingY_ErrEst);
	bb.setOffset(64);	bb.put(TA_VelAidingZ_ErrEst);
	bb.setOffset(68);	bb.put(TA_LeverArmX_ErrEst);
	bb.setOffset(72);	bb.put(TA_LeverArmY_ErrEst);
	bb.setOffset(76);	bb.put(TA_LeverArmZ_ErrEst);
	bb.setOffset(80);	bb.put(TA_BoresightX_ErrEst);
	bb.setOffset(84);	bb.put(TA_BoresightY_ErrEst);
	bb.setOffset(88);	bb.put(TA_BoresightZ_ErrEst);
	bb.setOffset(104);	bb.put(TA_PosAidingX_Stdv);
	bb.setOffset(108);	bb.put(TA_PosAidingY_Stdv);
	bb.setOffset(112);	bb.put(TA_PosAidingZ_Stdv);
	bb.setOffset(116);	bb.put(TA_VelAidingX_Stdv);
	bb.setOffset(120);	bb.put(TA_VelAidingY_Stdv);
	bb.setOffset(124);	bb.put(TA_VelAidingZ_Stdv);
	bb.setOffset(128);	bb.put(TA_LeverArmX_Stdv);
	bb.setOffset(132);	bb.put(TA_LeverArmY_Stdv);
	bb.setOffset(136);	bb.put(TA_LeverArmZ_Stdv);
	bb.setOffset(140);	bb.put(TA_BoresightX_Stdv);
	bb.setOffset(144);	bb.put(TA_BoresightY_Stdv);
	bb.setOffset(148);	bb.put(TA_BoresightZ_Stdv);
	Checksum = computeChecksum((uint32_t*)buffer, 41);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2425::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 164) return -2;

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
	bb.setOffset(20);	INSMode = static_cast<ins_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(TA_PosAidingX_ErrEst);
	bb.setOffset(48);	bb.get(TA_PosAidingY_ErrEst);
	bb.setOffset(52);	bb.get(TA_PosAidingZ_ErrEst);
	bb.setOffset(56);	bb.get(TA_VelAidingX_ErrEst);
	bb.setOffset(60);	bb.get(TA_VelAidingY_ErrEst);
	bb.setOffset(64);	bb.get(TA_VelAidingZ_ErrEst);
	bb.setOffset(68);	bb.get(TA_LeverArmX_ErrEst);
	bb.setOffset(72);	bb.get(TA_LeverArmY_ErrEst);
	bb.setOffset(76);	bb.get(TA_LeverArmZ_ErrEst);
	bb.setOffset(80);	bb.get(TA_BoresightX_ErrEst);
	bb.setOffset(84);	bb.get(TA_BoresightY_ErrEst);
	bb.setOffset(88);	bb.get(TA_BoresightZ_ErrEst);
	bb.setOffset(104);	bb.get(TA_PosAidingX_Stdv);
	bb.setOffset(108);	bb.get(TA_PosAidingY_Stdv);
	bb.setOffset(112);	bb.get(TA_PosAidingZ_Stdv);
	bb.setOffset(116);	bb.get(TA_VelAidingX_Stdv);
	bb.setOffset(120);	bb.get(TA_VelAidingY_Stdv);
	bb.setOffset(124);	bb.get(TA_VelAidingZ_Stdv);
	bb.setOffset(128);	bb.get(TA_LeverArmX_Stdv);
	bb.setOffset(132);	bb.get(TA_LeverArmY_Stdv);
	bb.setOffset(136);	bb.get(TA_LeverArmZ_Stdv);
	bb.setOffset(140);	bb.get(TA_BoresightX_Stdv);
	bb.setOffset(144);	bb.get(TA_BoresightY_Stdv);
	bb.setOffset(148);	bb.get(TA_BoresightZ_Stdv);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 41);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

