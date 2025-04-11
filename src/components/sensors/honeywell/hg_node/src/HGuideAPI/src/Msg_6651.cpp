#include <include/HGuideAPI.h>
#include <include/Msg_6651.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6651::AddressId;
const uint32_t Msg_6651::MessageId;
const uint32_t Msg_6651::MessageLength;

Msg_6651::Msg_6651()
{
	Default();
}

void Msg_6651::Default()
{
	Checksum = 0;
	ImuDataStatus.Default();
	GnssDatatStatus.Default();
	PositionStatus.Default();
	VelocityStatus.Default();
	AttitudeStatus.Default();
	NavAxesSelected = 0;
}

bool Msg_6651::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 64) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t ImuDataStatus_tmp = 0; // temporary variable holding the custom bitfield
	ImuDataStatus_tmp = ECTOS::BIT_UTILITIES::Pack(ImuDataStatus_tmp, 0, 7, ImuDataStatus.portNum, status_ok);
	ImuDataStatus_tmp = ECTOS::BIT_UTILITIES::Pack(ImuDataStatus_tmp, 8, 15, ImuDataStatus.portType, status_ok);
	ImuDataStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(ImuDataStatus_tmp, 16, ImuDataStatus.active, status_ok);
	bb.setOffset(16);	bb.put(ImuDataStatus_tmp);


	uint32_t GnssDatatStatus_tmp = 0; // temporary variable holding the custom bitfield
	GnssDatatStatus_tmp = ECTOS::BIT_UTILITIES::Pack(GnssDatatStatus_tmp, 0, 7, GnssDatatStatus.portNum, status_ok);
	GnssDatatStatus_tmp = ECTOS::BIT_UTILITIES::Pack(GnssDatatStatus_tmp, 8, 15, GnssDatatStatus.portType, status_ok);
	GnssDatatStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(GnssDatatStatus_tmp, 16, GnssDatatStatus.active, status_ok);
	bb.setOffset(20);	bb.put(GnssDatatStatus_tmp);


	uint32_t PositionStatus_tmp = 0; // temporary variable holding the custom bitfield
	PositionStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(PositionStatus_tmp, 0, PositionStatus.x_axis, status_ok);
	PositionStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(PositionStatus_tmp, 1, PositionStatus.y_axis, status_ok);
	PositionStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(PositionStatus_tmp, 2, PositionStatus.z_axis, status_ok);
	PositionStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(PositionStatus_tmp, 8, PositionStatus.enabled, status_ok);
	bb.setOffset(44);	bb.put(PositionStatus_tmp);


	uint32_t VelocityStatus_tmp = 0; // temporary variable holding the custom bitfield
	VelocityStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(VelocityStatus_tmp, 0, VelocityStatus.x_axis, status_ok);
	VelocityStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(VelocityStatus_tmp, 1, VelocityStatus.y_axis, status_ok);
	VelocityStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(VelocityStatus_tmp, 2, VelocityStatus.z_axis, status_ok);
	VelocityStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(VelocityStatus_tmp, 8, VelocityStatus.enabled, status_ok);
	bb.setOffset(48);	bb.put(VelocityStatus_tmp);


	uint32_t AttitudeStatus_tmp = 0; // temporary variable holding the custom bitfield
	AttitudeStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(AttitudeStatus_tmp, 0, AttitudeStatus.x_axis, status_ok);
	AttitudeStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(AttitudeStatus_tmp, 1, AttitudeStatus.y_axis, status_ok);
	AttitudeStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(AttitudeStatus_tmp, 2, AttitudeStatus.z_axis, status_ok);
	AttitudeStatus_tmp = ECTOS::BIT_UTILITIES::PackBool(AttitudeStatus_tmp, 8, AttitudeStatus.enabled, status_ok);
	bb.setOffset(52);	bb.put(AttitudeStatus_tmp);

	bb.setOffset(56);	bb.put(NavAxesSelected);
	Checksum = computeChecksum((uint32_t*)buffer, 16);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6651::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 64) return -2;

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

	uint32_t ImuDataStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(ImuDataStatus_tmp);
	ImuDataStatus.portNum = ECTOS::BIT_UTILITIES::UnPack(ImuDataStatus_tmp, 0, 7,  status_ok);
	ImuDataStatus.portType = static_cast<port_type_t>(ECTOS::BIT_UTILITIES::UnPack(ImuDataStatus_tmp, 8, 15,  status_ok));
	ImuDataStatus.active = ECTOS::BIT_UTILITIES::UnPackBool(ImuDataStatus_tmp, 16,  status_ok);

	uint32_t GnssDatatStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(20);	bb.get(GnssDatatStatus_tmp);
	GnssDatatStatus.portNum = ECTOS::BIT_UTILITIES::UnPack(GnssDatatStatus_tmp, 0, 7,  status_ok);
	GnssDatatStatus.portType = static_cast<port_type_t>(ECTOS::BIT_UTILITIES::UnPack(GnssDatatStatus_tmp, 8, 15,  status_ok));
	GnssDatatStatus.active = ECTOS::BIT_UTILITIES::UnPackBool(GnssDatatStatus_tmp, 16,  status_ok);

	uint32_t PositionStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(44);	bb.get(PositionStatus_tmp);
	PositionStatus.x_axis = ECTOS::BIT_UTILITIES::UnPackBool(PositionStatus_tmp, 0,  status_ok);
	PositionStatus.y_axis = ECTOS::BIT_UTILITIES::UnPackBool(PositionStatus_tmp, 1,  status_ok);
	PositionStatus.z_axis = ECTOS::BIT_UTILITIES::UnPackBool(PositionStatus_tmp, 2,  status_ok);
	PositionStatus.enabled = ECTOS::BIT_UTILITIES::UnPackBool(PositionStatus_tmp, 8,  status_ok);

	uint32_t VelocityStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(48);	bb.get(VelocityStatus_tmp);
	VelocityStatus.x_axis = ECTOS::BIT_UTILITIES::UnPackBool(VelocityStatus_tmp, 0,  status_ok);
	VelocityStatus.y_axis = ECTOS::BIT_UTILITIES::UnPackBool(VelocityStatus_tmp, 1,  status_ok);
	VelocityStatus.z_axis = ECTOS::BIT_UTILITIES::UnPackBool(VelocityStatus_tmp, 2,  status_ok);
	VelocityStatus.enabled = ECTOS::BIT_UTILITIES::UnPackBool(VelocityStatus_tmp, 8,  status_ok);

	uint32_t AttitudeStatus_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(52);	bb.get(AttitudeStatus_tmp);
	AttitudeStatus.x_axis = ECTOS::BIT_UTILITIES::UnPackBool(AttitudeStatus_tmp, 0,  status_ok);
	AttitudeStatus.y_axis = ECTOS::BIT_UTILITIES::UnPackBool(AttitudeStatus_tmp, 1,  status_ok);
	AttitudeStatus.z_axis = ECTOS::BIT_UTILITIES::UnPackBool(AttitudeStatus_tmp, 2,  status_ok);
	AttitudeStatus.enabled = ECTOS::BIT_UTILITIES::UnPackBool(AttitudeStatus_tmp, 8,  status_ok);
	bb.setOffset(56);	bb.get(NavAxesSelected);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 16);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

