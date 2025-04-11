#include <include/HGuideAPI.h>
#include <include/Msg_1401.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1401::AddressId;
const uint32_t Msg_1401::MessageId;
const uint32_t Msg_1401::MessageLength;

Msg_1401::Msg_1401()
{
	Default();
}

void Msg_1401::Default()
{
	Checksum = 0;
	RequestACKNAKReply = 1;
	PositionTov = 0;
	PositionValidity = 0;
	PositionTimeReferenceMode = 0;
	PositionCoordinateFrame = 0;
	PositionStdvValidity = 0;
	Latitude = 0;
	Longitude = 0;
	AltitudeHeightAboveEllipsoid = 0;
	EcefPositionX = 0;
	EcefPositionY = 0;
	EcefPositionZ = 0;
	VelocityTov = 0;
	VelocityValidity = 0;
	VelocityTimeReferenceMode = 0;
	VelocityCoordinateFrame = 0;
	VelocityStdvValidity = 0;
	VelocityNorthOrEcefVelocityX = 0;
	VelocityEastOrEcefVelocityY = 0;
	VelocityDownOrEcefVelocityZ = 0;
	AttitudeTov = 0;
	AttitudeValidity = 0;
	AttitudeTimeReferenceMode = 0;
	AttitudeCoordinateFrame = 0;
	AttitudeStdvValidity = 0;
	DCM11 = 0;
	DCM12 = 0;
	DCM13 = 0;
	DCM21 = 0;
	DCM22 = 0;
	DCM23 = 0;
	DCM31 = 0;
	DCM32 = 0;
	DCM33 = 0;
	EulerAnglesRoll = 0;
	EulerAnglesPitch = 0;
	EulerAnglesTrueHeading = 0;
	PositionStdvNorth = 100.0;
	PositionStdvEast = 100.0;
	PositionStdvDown = 100.0;
	VelocityStdvNorth = 1.0;
	VelocityStdvEast = 1.0;
	VelocityStdvDown = 1.0;
	EulerAnglesStdvRoll = 0.0873;
	EulerAnglesStdvPitch = 0.0873;
	EulerAnglesStdvTrueHeading = 0.0873;
}

bool Msg_1401::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 156) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 0, RequestACKNAKReply, status_ok);
	bb.setOffset(16);	bb.put(bitfieldAtByte16);

	bb.setOffset(24);	bb.put(PositionTov);

	uint8_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 0, PositionValidity, status_ok);
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 1, PositionTimeReferenceMode, status_ok);
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 2, PositionCoordinateFrame, status_ok);
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 3, PositionStdvValidity, status_ok);
	bb.setOffset(32);	bb.put(bitfieldAtByte32);

	bb.setOffset(48);	bb.put(VelocityTov);

	uint8_t bitfieldAtByte56 = 0; // temporary variable holding the bitfield
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte56, 0, VelocityValidity, status_ok);
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte56, 1, VelocityTimeReferenceMode, status_ok);
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte56, 2, VelocityCoordinateFrame, status_ok);
	bitfieldAtByte56 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte56, 3, VelocityStdvValidity, status_ok);
	bb.setOffset(56);	bb.put(bitfieldAtByte56);

	bb.setOffset(60);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityNorthOrEcefVelocityX / (std::pow(2, -17))));
	bb.setOffset(64);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityEastOrEcefVelocityY / (std::pow(2, -17))));
	bb.setOffset(68);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityDownOrEcefVelocityZ / (std::pow(2, -17))));
	bb.setOffset(72);	bb.put(AttitudeTov);

	uint8_t bitfieldAtByte80 = 0; // temporary variable holding the bitfield
	bitfieldAtByte80 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte80, 0, AttitudeValidity, status_ok);
	bitfieldAtByte80 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte80, 1, AttitudeTimeReferenceMode, status_ok);
	bitfieldAtByte80 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte80, 2, AttitudeCoordinateFrame, status_ok);
	bitfieldAtByte80 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte80, 3, AttitudeStdvValidity, status_ok);
	bb.setOffset(80);	bb.put(bitfieldAtByte80);

	bb.setOffset(120);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(PositionStdvNorth / (std::pow(2, -7))));
	bb.setOffset(124);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(PositionStdvEast / (std::pow(2, -7))));
	bb.setOffset(128);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(PositionStdvDown / (std::pow(2, -7))));
	bb.setOffset(132);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityStdvNorth / (std::pow(2, -17))));
	bb.setOffset(136);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityStdvEast / (std::pow(2, -17))));
	bb.setOffset(140);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(VelocityStdvDown / (std::pow(2, -17))));
	bb.setOffset(144);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesStdvRoll / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
	bb.setOffset(148);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesStdvPitch / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
	bb.setOffset(152);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesStdvTrueHeading / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));

	if ( PositionCoordinateFrame == 0 ) {
			bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Latitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(Longitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(AltitudeHeightAboveEllipsoid / (std::pow(2, -14))));
		}

	if ( PositionCoordinateFrame == 1 ) {
			bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionX / (std::pow(2, -7))));
			bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionY / (std::pow(2, -7))));
			bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EcefPositionZ / (std::pow(2, -7))));
		}

	if ( AttitudeCoordinateFrame == 0 ) {
			bb.setOffset(84);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM11 / (std::pow(2, -31))));
			bb.setOffset(88);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM12 / (std::pow(2, -31))));
			bb.setOffset(92);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM13 / (std::pow(2, -31))));
			bb.setOffset(96);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM21 / (std::pow(2, -31))));
			bb.setOffset(100);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM22 / (std::pow(2, -31))));
			bb.setOffset(104);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM23 / (std::pow(2, -31))));
			bb.setOffset(108);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM31 / (std::pow(2, -31))));
			bb.setOffset(112);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM32 / (std::pow(2, -31))));
			bb.setOffset(116);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DCM33 / (std::pow(2, -31))));
		}

	if ( AttitudeCoordinateFrame == 1 ) {
			bb.setOffset(84);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesRoll / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(88);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesPitch / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(92);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(EulerAnglesTrueHeading / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
		}
	Checksum = computeChecksum((uint32_t*)buffer, 39);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1401::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 156) return -2;

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

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bb.setOffset(16);	bb.get(bitfieldAtByte16);
	RequestACKNAKReply = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 0,  status_ok);
	bb.setOffset(24);	bb.get(PositionTov);

	uint8_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bb.setOffset(32);	bb.get(bitfieldAtByte32);
	PositionValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 0,  status_ok);
	PositionTimeReferenceMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 1,  status_ok);
	PositionCoordinateFrame = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 2,  status_ok);
	PositionStdvValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 3,  status_ok);
	bb.setOffset(48);	bb.get(VelocityTov);

	uint8_t bitfieldAtByte56 = 0; // temporary variable holding the bitfield
	bb.setOffset(56);	bb.get(bitfieldAtByte56);
	VelocityValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte56, 0,  status_ok);
	VelocityTimeReferenceMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte56, 1,  status_ok);
	VelocityCoordinateFrame = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte56, 2,  status_ok);
	VelocityStdvValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte56, 3,  status_ok);
	bb.setOffset(60);	VelocityNorthOrEcefVelocityX = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(64);	VelocityEastOrEcefVelocityY = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(68);	VelocityDownOrEcefVelocityZ = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(72);	bb.get(AttitudeTov);

	uint8_t bitfieldAtByte80 = 0; // temporary variable holding the bitfield
	bb.setOffset(80);	bb.get(bitfieldAtByte80);
	AttitudeValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte80, 0,  status_ok);
	AttitudeTimeReferenceMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte80, 1,  status_ok);
	AttitudeCoordinateFrame = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte80, 2,  status_ok);
	AttitudeStdvValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte80, 3,  status_ok);
	bb.setOffset(120);	PositionStdvNorth = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(124);	PositionStdvEast = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(128);	PositionStdvDown = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
	bb.setOffset(132);	VelocityStdvNorth = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(136);	VelocityStdvEast = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(140);	VelocityStdvDown = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -17));
	bb.setOffset(144);	EulerAnglesStdvRoll = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
	bb.setOffset(148);	EulerAnglesStdvPitch = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
	bb.setOffset(152);	EulerAnglesStdvTrueHeading = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));

	if ( PositionCoordinateFrame == 0 ) {
		 	bb.setOffset(36);	Latitude = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(40);	Longitude = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(44);	AltitudeHeightAboveEllipsoid = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -14));
}

	if ( PositionCoordinateFrame == 1 ) {
		 	bb.setOffset(36);	EcefPositionX = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
		 	bb.setOffset(40);	EcefPositionY = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
		 	bb.setOffset(44);	EcefPositionZ = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -7));
}

	if ( AttitudeCoordinateFrame == 0 ) {
		 	bb.setOffset(84);	DCM11 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(88);	DCM12 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(92);	DCM13 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(96);	DCM21 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(100);	DCM22 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(104);	DCM23 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(108);	DCM31 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(112);	DCM32 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
		 	bb.setOffset(116);	DCM33 = static_cast<float>(bb.get<int32_t>()) * (std::pow(2, -31));
}

	if ( AttitudeCoordinateFrame == 1 ) {
		 	bb.setOffset(84);	EulerAnglesRoll = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(88);	EulerAnglesPitch = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(92);	EulerAnglesTrueHeading = static_cast<float>(bb.get<int32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 39);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

