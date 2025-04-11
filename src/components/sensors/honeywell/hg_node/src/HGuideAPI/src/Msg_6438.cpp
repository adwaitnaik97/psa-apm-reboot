#include <include/HGuideAPI.h>
#include <include/Msg_6438.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6438::AddressId;
const uint32_t Msg_6438::MessageId;
const uint32_t Msg_6438::MessageLength;

Msg_6438::Msg_6438()
{
	Default();
}

void Msg_6438::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	residual_x = 0;
	residual_y = 0;
	residual_z = 0;
	scalefactor_correction = 0;
	scalefactor_correction_pitchsens = 0;
	scalefactor_correction_pitchratesens = 0;
	boresight_yaw = 0;
	boresight_yaw_rollsens = 0;
	boresight_yaw_rocksens = 0;
	boresight_pitch = 0;
	boresight_pitch_rollsens = 0;
	boresight_pitch_rocksens = 0;
	lever_arm_x = 0;
	lever_arm_y = 0;
	lever_arm_z = 0;
	scalefactor_correction_stdv = 0;
	scalefactor_correction_pitchsens_stdv = 0;
	scalefactor_correction_pitchratesens_stdv = 0;
	boresight_yaw_stdv = 0;
	boresight_yaw_rollsens_stdv = 0;
	boresight_yaw_rocksens_stdv = 0;
	boresight_pitch_stdv = 0;
	boresight_pitch_rollsens_stdv = 0;
	boresight_pitch_rocksens_stdv = 0;
	lever_arm_stdv_x = 0;
	lever_arm_stdv_y = 0;
	lever_arm_stdv_z = 0;
	stored_lever_arm_x = 0;
	stored_lever_arm_y = 0;
	stored_lever_arm_z = 0;
	boresight_roll = 0;
	boresight_stdv_roll = 0;
}

bool Msg_6438::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(24);	bb.put(systemTov);
	bb.setOffset(32);	bb.put(gpsTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(44);	bb.put(residual_x);
	bb.setOffset(48);	bb.put(residual_y);
	bb.setOffset(52);	bb.put(residual_z);
	bb.setOffset(56);	bb.put(scalefactor_correction);
	bb.setOffset(60);	bb.put(scalefactor_correction_pitchsens);
	bb.setOffset(64);	bb.put(scalefactor_correction_pitchratesens);
	bb.setOffset(68);	bb.put(boresight_yaw);
	bb.setOffset(72);	bb.put(boresight_yaw_rollsens);
	bb.setOffset(76);	bb.put(boresight_yaw_rocksens);
	bb.setOffset(80);	bb.put(boresight_pitch);
	bb.setOffset(84);	bb.put(boresight_pitch_rollsens);
	bb.setOffset(88);	bb.put(boresight_pitch_rocksens);
	bb.setOffset(92);	bb.put(lever_arm_x);
	bb.setOffset(96);	bb.put(lever_arm_y);
	bb.setOffset(100);	bb.put(lever_arm_z);
	bb.setOffset(104);	bb.put(scalefactor_correction_stdv);
	bb.setOffset(108);	bb.put(scalefactor_correction_pitchsens_stdv);
	bb.setOffset(112);	bb.put(scalefactor_correction_pitchratesens_stdv);
	bb.setOffset(116);	bb.put(boresight_yaw_stdv);
	bb.setOffset(116);	bb.put(boresight_yaw_rollsens_stdv);
	bb.setOffset(116);	bb.put(boresight_yaw_rocksens_stdv);
	bb.setOffset(128);	bb.put(boresight_pitch_stdv);
	bb.setOffset(132);	bb.put(boresight_pitch_rollsens_stdv);
	bb.setOffset(136);	bb.put(boresight_pitch_rocksens_stdv);
	bb.setOffset(140);	bb.put(lever_arm_stdv_x);
	bb.setOffset(144);	bb.put(lever_arm_stdv_y);
	bb.setOffset(148);	bb.put(lever_arm_stdv_z);
	bb.setOffset(152);	bb.put(stored_lever_arm_x);
	bb.setOffset(156);	bb.put(stored_lever_arm_y);
	bb.setOffset(160);	bb.put(stored_lever_arm_z);
	bb.setOffset(164);	bb.put(boresight_roll);
	bb.setOffset(168);	bb.put(boresight_stdv_roll);
	Checksum = computeChecksum((uint32_t*)buffer, 48);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6438::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(24);	bb.get(systemTov);
	bb.setOffset(32);	bb.get(gpsTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(44);	bb.get(residual_x);
	bb.setOffset(48);	bb.get(residual_y);
	bb.setOffset(52);	bb.get(residual_z);
	bb.setOffset(56);	bb.get(scalefactor_correction);
	bb.setOffset(60);	bb.get(scalefactor_correction_pitchsens);
	bb.setOffset(64);	bb.get(scalefactor_correction_pitchratesens);
	bb.setOffset(68);	bb.get(boresight_yaw);
	bb.setOffset(72);	bb.get(boresight_yaw_rollsens);
	bb.setOffset(76);	bb.get(boresight_yaw_rocksens);
	bb.setOffset(80);	bb.get(boresight_pitch);
	bb.setOffset(84);	bb.get(boresight_pitch_rollsens);
	bb.setOffset(88);	bb.get(boresight_pitch_rocksens);
	bb.setOffset(92);	bb.get(lever_arm_x);
	bb.setOffset(96);	bb.get(lever_arm_y);
	bb.setOffset(100);	bb.get(lever_arm_z);
	bb.setOffset(104);	bb.get(scalefactor_correction_stdv);
	bb.setOffset(108);	bb.get(scalefactor_correction_pitchsens_stdv);
	bb.setOffset(112);	bb.get(scalefactor_correction_pitchratesens_stdv);
	bb.setOffset(116);	bb.get(boresight_yaw_stdv);
	bb.setOffset(116);	bb.get(boresight_yaw_rollsens_stdv);
	bb.setOffset(116);	bb.get(boresight_yaw_rocksens_stdv);
	bb.setOffset(128);	bb.get(boresight_pitch_stdv);
	bb.setOffset(132);	bb.get(boresight_pitch_rollsens_stdv);
	bb.setOffset(136);	bb.get(boresight_pitch_rocksens_stdv);
	bb.setOffset(140);	bb.get(lever_arm_stdv_x);
	bb.setOffset(144);	bb.get(lever_arm_stdv_y);
	bb.setOffset(148);	bb.get(lever_arm_stdv_z);
	bb.setOffset(152);	bb.get(stored_lever_arm_x);
	bb.setOffset(156);	bb.get(stored_lever_arm_y);
	bb.setOffset(160);	bb.get(stored_lever_arm_z);
	bb.setOffset(164);	bb.get(boresight_roll);
	bb.setOffset(168);	bb.get(boresight_stdv_roll);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 48);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

