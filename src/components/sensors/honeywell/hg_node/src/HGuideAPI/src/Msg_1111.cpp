#include <include/HGuideAPI.h>
#include <include/Msg_1111.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1111::AddressId;
const uint32_t Msg_1111::MessageId;
const uint32_t Msg_1111::MessageLength;

Msg_1111::Msg_1111()
{
	Default();
}

void Msg_1111::Default()
{
	Checksum = 0;
	trajectory_aiding_validity.Default();
	ESpaceTrajectoryTOV = 0;
	Latitude = 0;
	Longitude = 0;
	Altitude_height_above_ellipsoid = 0;
	Baro_altitude = 0;
	True_air_speed = 0;
	velocity_north = 0;
	velocity_east = 0;
	velocity_down = 0;
	velocity_x = 0;
	velocity_y = 0;
	velocity_z = 0;
	attitude_roll = 0;
	attitude_pitch = 0;
	attitude_heading = 0;
}

bool Msg_1111::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 120) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t trajectory_aiding_validity_tmp = 0; // temporary variable holding the custom bitfield
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 0, trajectory_aiding_validity.LATITUDE, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 1, trajectory_aiding_validity.LONGITUDE, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 2, trajectory_aiding_validity.ALTITUDE, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 3, trajectory_aiding_validity.BARO_ALTITUDE, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 4, trajectory_aiding_validity.TRUE_AIR_SPEED, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 5, trajectory_aiding_validity.VELOCITY_NORTH, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 6, trajectory_aiding_validity.VELOCITY_EAST, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 7, trajectory_aiding_validity.VELOCITY_DOWN, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 8, trajectory_aiding_validity.VELOCITY_X, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 9, trajectory_aiding_validity.VELOCITY_Y, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 10, trajectory_aiding_validity.VELOCITY_Z, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 11, trajectory_aiding_validity.ATTITUDE_ROLL, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 12, trajectory_aiding_validity.ATTITUDE_PITCH, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 13, trajectory_aiding_validity.ATTITUDE_HEADING, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 24, trajectory_aiding_validity.ALTITUDE_TYPE, status_ok);
	trajectory_aiding_validity_tmp = ECTOS::BIT_UTILITIES::PackBool(trajectory_aiding_validity_tmp, 25, trajectory_aiding_validity.HEADING_TYPE, status_ok);
	bb.setOffset(16);	bb.put(trajectory_aiding_validity_tmp);

	bb.setOffset(24);	bb.put(ESpaceTrajectoryTOV);
	bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Latitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
	bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Longitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
	bb.setOffset(48);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Altitude_height_above_ellipsoid / (std::pow(2, -14))));
	bb.setOffset(52);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Baro_altitude / (std::pow(2, -14))));
	bb.setOffset(56);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(True_air_speed / (std::pow(2, -17))));
	bb.setOffset(60);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_north / (std::pow(2, -17))));
	bb.setOffset(64);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_east / (std::pow(2, -17))));
	bb.setOffset(68);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_down / (std::pow(2, -17))));
	bb.setOffset(72);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_x / (std::pow(2, -17))));
	bb.setOffset(76);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_y / (std::pow(2, -17))));
	bb.setOffset(80);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(velocity_z / (std::pow(2, -17))));
	bb.setOffset(84);	bb.put(attitude_roll);
	bb.setOffset(88);	bb.put(attitude_pitch);
	bb.setOffset(92);	bb.put(attitude_heading);
	Checksum = computeChecksum((uint32_t*)buffer, 30);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1111::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 120) return -2;

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

	uint32_t trajectory_aiding_validity_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(trajectory_aiding_validity_tmp);
	trajectory_aiding_validity.LATITUDE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 0,  status_ok);
	trajectory_aiding_validity.LONGITUDE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 1,  status_ok);
	trajectory_aiding_validity.ALTITUDE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 2,  status_ok);
	trajectory_aiding_validity.BARO_ALTITUDE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 3,  status_ok);
	trajectory_aiding_validity.TRUE_AIR_SPEED = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 4,  status_ok);
	trajectory_aiding_validity.VELOCITY_NORTH = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 5,  status_ok);
	trajectory_aiding_validity.VELOCITY_EAST = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 6,  status_ok);
	trajectory_aiding_validity.VELOCITY_DOWN = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 7,  status_ok);
	trajectory_aiding_validity.VELOCITY_X = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 8,  status_ok);
	trajectory_aiding_validity.VELOCITY_Y = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 9,  status_ok);
	trajectory_aiding_validity.VELOCITY_Z = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 10,  status_ok);
	trajectory_aiding_validity.ATTITUDE_ROLL = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 11,  status_ok);
	trajectory_aiding_validity.ATTITUDE_PITCH = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 12,  status_ok);
	trajectory_aiding_validity.ATTITUDE_HEADING = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 13,  status_ok);
	trajectory_aiding_validity.ALTITUDE_TYPE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 24,  status_ok);
	trajectory_aiding_validity.HEADING_TYPE = ECTOS::BIT_UTILITIES::UnPackBool(trajectory_aiding_validity_tmp, 25,  status_ok);
	bb.setOffset(24);	bb.get(ESpaceTrajectoryTOV);
	bb.setOffset(40);	Latitude = static_cast<double>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
	bb.setOffset(44);	Longitude = static_cast<double>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
	bb.setOffset(48);	Altitude_height_above_ellipsoid = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -14));
	bb.setOffset(52);	Baro_altitude = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -14));
	bb.setOffset(56);	True_air_speed = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(60);	velocity_north = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(64);	velocity_east = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(68);	velocity_down = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(72);	velocity_x = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(76);	velocity_y = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(80);	velocity_z = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(84);	bb.get(attitude_roll);
	bb.setOffset(88);	bb.get(attitude_pitch);
	bb.setOffset(92);	bb.get(attitude_heading);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 30);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

