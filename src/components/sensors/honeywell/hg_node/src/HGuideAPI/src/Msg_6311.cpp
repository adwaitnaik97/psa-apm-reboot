#include <include/HGuideAPI.h>
#include <include/Msg_6311.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6311::AddressId;
const uint32_t Msg_6311::MessageId;
const uint32_t Msg_6311::MessageLength;

Msg_6311::Msg_6311()
{
	Default();
}

void Msg_6311::Default()
{
	Checksum = 0;
	systemTov = 0;
	InsGnssSummary.Default();
	delta_theta_x = 0;
	delta_theta_y = 0;
	delta_theta_z = 0;
	gps_week = 0;
	delta_velocity_x = 0;
	delta_velocity_y = 0;
	delta_velocity_z = 0;
	gpsTov = 0;
	current_coordinate_frame = static_cast<coordinate_frame_t>(0);
}

bool Msg_6311::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 72) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 0, 3, InsGnssSummary.INSMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 4, InsGnssSummary.INSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 5, InsGnssSummary.IMUStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 6, InsGnssSummary.GNSSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 7, InsGnssSummary.MotionDetectActive, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 8, InsGnssSummary.StationaryMeasurementsOn, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 9, InsGnssSummary.MDT1RotationRate, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 10, InsGnssSummary.MDT2SpeedSTDV, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 11, InsGnssSummary.MDT3AngularRateInstantBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 12, InsGnssSummary.MDT4LinearAccelerationBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 13, InsGnssSummary.MDT5OdometerBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 16, InsGnssSummary.MDNavigationMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 17, 19, InsGnssSummary.NavSmoothingStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 28, 31, InsGnssSummary.GPSMode, status_ok);
	bb.setOffset(24);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(28);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_theta_x / (std::pow(2, -33))));
	bb.setOffset(32);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_theta_y / (std::pow(2, -33))));
	bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_theta_z / (std::pow(2, -33))));
	bb.setOffset(44);	bb.put(gps_week);
	bb.setOffset(48);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_velocity_x / (std::pow(2, -29))));
	bb.setOffset(52);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_velocity_y / (std::pow(2, -29))));
	bb.setOffset(56);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(delta_velocity_z / (std::pow(2, -29))));
	bb.setOffset(60);	bb.put(gpsTov);

	uint8_t bitfieldAtByte68 = 0; // temporary variable holding the bitfield
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte68, 0, 3, static_cast<uint8_t>(current_coordinate_frame), status_ok);
	bb.setOffset(68);	bb.put(bitfieldAtByte68);

	Checksum = computeChecksum((uint32_t*)buffer, 18);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6311::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 72) return -2;

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

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(24);	bb.get(InsGnssSummary_tmp);
	InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 0, 3,  status_ok));
	InsGnssSummary.INSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 4,  status_ok);
	InsGnssSummary.IMUStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 5,  status_ok);
	InsGnssSummary.GNSSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 6,  status_ok);
	InsGnssSummary.MotionDetectActive = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 7,  status_ok);
	InsGnssSummary.StationaryMeasurementsOn = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 8,  status_ok);
	InsGnssSummary.MDT1RotationRate = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 9,  status_ok);
	InsGnssSummary.MDT2SpeedSTDV = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 10,  status_ok);
	InsGnssSummary.MDT3AngularRateInstantBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 11,  status_ok);
	InsGnssSummary.MDT4LinearAccelerationBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 12,  status_ok);
	InsGnssSummary.MDT5OdometerBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 13,  status_ok);
	InsGnssSummary.MDNavigationMode = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 16,  status_ok);
	InsGnssSummary.NavSmoothingStatus = ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 17, 19,  status_ok);
	InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 28, 31,  status_ok));
	bb.setOffset(28);	delta_theta_x = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(32);	delta_theta_y = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(36);	delta_theta_z = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(44);	bb.get(gps_week);
	bb.setOffset(48);	delta_velocity_x = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -29));
	bb.setOffset(52);	delta_velocity_y = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -29));
	bb.setOffset(56);	delta_velocity_z = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -29));
	bb.setOffset(60);	bb.get(gpsTov);

	uint8_t bitfieldAtByte68 = 0; // temporary variable holding the bitfield
	bb.setOffset(68);	bb.get(bitfieldAtByte68);
	current_coordinate_frame = static_cast<coordinate_frame_t>(ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte68, 0, 3,  status_ok));
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 18);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

