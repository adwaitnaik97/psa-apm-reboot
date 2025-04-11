#include <include/HGuideAPI.h>
#include <include/Msg_2301.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2301::AddressId;
const uint32_t Msg_2301::MessageId;
const uint32_t Msg_2301::MessageLength;

Msg_2301::Msg_2301()
{
	Default();
}

void Msg_2301::Default()
{
	Checksum = 0;
	systemTov = 0;
	InsGnssSummary.Default();
	angular_rate_x = 0;
	angular_rate_y = 0;
	angular_rate_z = 0;
	linear_acceleration_x = 0;
	linear_acceleration_y = 0;
	linear_acceleration_z = 0;
	outputFrame = 0;
}

bool Msg_2301::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 60) return false;

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
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 17, 19, InsGnssSummary.SnapbackStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 28, 31, InsGnssSummary.GPSMode, status_ok);
	bb.setOffset(24);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(28);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(angular_rate_x / (std::pow(2, -23))));
	bb.setOffset(32);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(angular_rate_y / (std::pow(2, -23))));
	bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(angular_rate_z / (std::pow(2, -23))));
	bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(linear_acceleration_x / (std::pow(2, -21))));
	bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(linear_acceleration_y / (std::pow(2, -21))));
	bb.setOffset(48);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(linear_acceleration_z / (std::pow(2, -21))));
	bb.setOffset(52);	bb.put(outputFrame);
	Checksum = computeChecksum((uint32_t*)buffer, 15);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2301::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 60) return -2;

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
	InsGnssSummary.SnapbackStatus = ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 17, 19,  status_ok);
	InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 28, 31,  status_ok));
	bb.setOffset(28);	angular_rate_x = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -23));
	bb.setOffset(32);	angular_rate_y = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -23));
	bb.setOffset(36);	angular_rate_z = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -23));
	bb.setOffset(40);	linear_acceleration_x = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -21));
	bb.setOffset(44);	linear_acceleration_y = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -21));
	bb.setOffset(48);	linear_acceleration_z = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -21));
	bb.setOffset(52);	bb.get(outputFrame);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 15);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

