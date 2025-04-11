#include <include/HGuideAPI.h>
#include <include/Msg_2401.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2401::AddressId;
const uint32_t Msg_2401::MessageId;
const uint32_t Msg_2401::MessageLength;

Msg_2401::Msg_2401()
{
	Default();
}

void Msg_2401::Default()
{
	Checksum = 0;
	InsGnssSummary = 0;
	INSMode = static_cast<ins_mode_table_t>(0);
	gpsTov = 0;
	systemTov = 0;
	gps_week = 0;
	utc_time_figure_of_merit = 0;
	gps_figure_of_merit = 0;
	ins_blended_figure_of_merit = 0;
	PositionTov = 0;
	Latitude = 0;
	Longitude = 0;
	AltitudeHeightAboveEllipsoid = 0;
	AltitudeMeanSeaLevel = 0;
	EcefPositionX = 0;
	EcefPositionY = 0;
	EcefPositionZ = 0;
	VelocityTov = 0;
	NorthVelocity = 0;
	EastVelocity = 0;
	DownVelocity = 0;
	EcefVelocityX = 0;
	EcefVelocityY = 0;
	EcefVelocityZ = 0;
	AttitudeTov = 0;
	EulerAnglesRoll = 0;
	EulerAnglesPitch = 0;
	EulerAnglesTrueHeading = 0;
	wander_angle = 0;
	DCM12 = 0;
	DCM13 = 0;
	DCM21 = 0;
	DCM23 = 0;
	DCM31 = 0;
	DCM32 = 0;
	angular_rate_x = 0;
	angular_rate_y = 0;
	angular_rate_z = 0;
	linear_acceleration_x = 0;
	linear_acceleration_y = 0;
	linear_acceleration_z = 0;
	attitude_figure_of_merit = 0;
	q1_vehicle_body_to_ecef = 0;
	q2_vehicle_body_to_ecef = 0;
	q3_vehicle_body_to_ecef = 0;
	DCM11 = 1.0;
	DCM22 = 1.0;
	DCM33 = 1.0;
	q0_vehicle_body_to_ecef = 1.0;
}

bool Msg_2401::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.put(InsGnssSummary);
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INSMode));
	bb.setOffset(24);	bb.put(gpsTov);
	bb.setOffset(32);	bb.put(systemTov);
	bb.setOffset(40);	bb.put(gps_week);
	bb.setOffset(42);	bb.put(utc_time_figure_of_merit);
	bb.setOffset(44);	bb.put(gps_figure_of_merit);
	bb.setOffset(46);	bb.put(ins_blended_figure_of_merit);
	bb.setOffset(48);	bb.put(PositionTov);
	bb.setOffset(56);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Latitude / (1.462918E-09)));
	bb.setOffset(60);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Longitude / (1.462918E-09)));
	bb.setOffset(64);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(AltitudeHeightAboveEllipsoid / (std::pow(2, -14))));
	bb.setOffset(68);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(AltitudeMeanSeaLevel / (std::pow(2, -14))));
	bb.setOffset(72);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionX / (std::pow(2, -7))));
	bb.setOffset(76);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionY / (std::pow(2, -7))));
	bb.setOffset(80);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionZ / (std::pow(2, -7))));
	bb.setOffset(88);	bb.put(VelocityTov);
	bb.setOffset(96);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(NorthVelocity / (std::pow(2, -17))));
	bb.setOffset(100);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EastVelocity / (std::pow(2, -17))));
	bb.setOffset(104);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DownVelocity / (std::pow(2, -17))));
	bb.setOffset(108);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefVelocityX / (std::pow(2, -17))));
	bb.setOffset(112);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefVelocityY / (std::pow(2, -17))));
	bb.setOffset(116);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefVelocityZ / (std::pow(2, -17))));
	bb.setOffset(120);	bb.put(AttitudeTov);
	bb.setOffset(128);	bb.put(EulerAnglesRoll);
	bb.setOffset(132);	bb.put(EulerAnglesPitch);
	bb.setOffset(136);	bb.put(EulerAnglesTrueHeading);
	bb.setOffset(140);	bb.put(wander_angle);
	bb.setOffset(144);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM11 / (std::pow(2, -31))));
	bb.setOffset(148);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM12 / (std::pow(2, -31))));
	bb.setOffset(152);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM13 / (std::pow(2, -31))));
	bb.setOffset(156);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM21 / (std::pow(2, -31))));
	bb.setOffset(160);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM22 / (std::pow(2, -31))));
	bb.setOffset(164);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM23 / (std::pow(2, -31))));
	bb.setOffset(168);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM31 / (std::pow(2, -31))));
	bb.setOffset(172);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM32 / (std::pow(2, -31))));
	bb.setOffset(176);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM33 / (std::pow(2, -31))));
	bb.setOffset(180);	bb.put(angular_rate_x);
	bb.setOffset(184);	bb.put(angular_rate_y);
	bb.setOffset(188);	bb.put(angular_rate_z);
	bb.setOffset(192);	bb.put(linear_acceleration_x);
	bb.setOffset(196);	bb.put(linear_acceleration_y);
	bb.setOffset(200);	bb.put(linear_acceleration_z);
	bb.setOffset(204);	bb.put(attitude_figure_of_merit);
	bb.setOffset(208);	bb.put(q0_vehicle_body_to_ecef);
	bb.setOffset(212);	bb.put(q1_vehicle_body_to_ecef);
	bb.setOffset(216);	bb.put(q2_vehicle_body_to_ecef);
	bb.setOffset(220);	bb.put(q3_vehicle_body_to_ecef);
	Checksum = computeChecksum((uint32_t*)buffer, 57);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2401::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.get(InsGnssSummary);
	bb.setOffset(20);	INSMode = static_cast<ins_mode_table_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(gpsTov);
	bb.setOffset(32);	bb.get(systemTov);
	bb.setOffset(40);	bb.get(gps_week);
	bb.setOffset(42);	bb.get(utc_time_figure_of_merit);
	bb.setOffset(44);	bb.get(gps_figure_of_merit);
	bb.setOffset(46);	bb.get(ins_blended_figure_of_merit);
	bb.setOffset(48);	bb.get(PositionTov);
	bb.setOffset(56);	Latitude = static_cast<double>(bb.get<int32_t>()) * (1.462918E-09);
	bb.setOffset(60);	Longitude = static_cast<double>(bb.get<int32_t>()) * (1.462918E-09);
	bb.setOffset(64);	AltitudeHeightAboveEllipsoid = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -14));
	bb.setOffset(68);	AltitudeMeanSeaLevel = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -14));
	bb.setOffset(72);	EcefPositionX = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(76);	EcefPositionY = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(80);	EcefPositionZ = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(88);	bb.get(VelocityTov);
	bb.setOffset(96);	NorthVelocity = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(100);	EastVelocity = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(104);	DownVelocity = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(108);	EcefVelocityX = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(112);	EcefVelocityY = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(116);	EcefVelocityZ = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(120);	bb.get(AttitudeTov);
	bb.setOffset(128);	bb.get(EulerAnglesRoll);
	bb.setOffset(132);	bb.get(EulerAnglesPitch);
	bb.setOffset(136);	bb.get(EulerAnglesTrueHeading);
	bb.setOffset(140);	bb.get(wander_angle);
	bb.setOffset(144);	DCM11 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(148);	DCM12 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(152);	DCM13 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(156);	DCM21 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(160);	DCM22 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(164);	DCM23 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(168);	DCM31 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(172);	DCM32 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(176);	DCM33 = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -31));
	bb.setOffset(180);	bb.get(angular_rate_x);
	bb.setOffset(184);	bb.get(angular_rate_y);
	bb.setOffset(188);	bb.get(angular_rate_z);
	bb.setOffset(192);	bb.get(linear_acceleration_x);
	bb.setOffset(196);	bb.get(linear_acceleration_y);
	bb.setOffset(200);	bb.get(linear_acceleration_z);
	bb.setOffset(204);	bb.get(attitude_figure_of_merit);
	bb.setOffset(208);	bb.get(q0_vehicle_body_to_ecef);
	bb.setOffset(212);	bb.get(q1_vehicle_body_to_ecef);
	bb.setOffset(216);	bb.get(q2_vehicle_body_to_ecef);
	bb.setOffset(220);	bb.get(q3_vehicle_body_to_ecef);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 57);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

