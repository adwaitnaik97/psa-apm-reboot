#include <include/HGuideAPI.h>
#include <include/Msg_1108.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1108::AddressId;
const uint32_t Msg_1108::MessageId;
const uint32_t Msg_1108::MessageLength;

Msg_1108::Msg_1108()
{
	Default();
}

void Msg_1108::Default()
{
	Checksum = 0;
	RequestAckNak = 0;
	TimeReferenceMode = 0;
	UnfilteredPointSolution = 0;
	GnssMode = 0;
	gpsPvtTov = 0;
	PositionValidity = 0;
	PositionCoordinateFrame = 0;
	Latitude = 0;
	Longitude = 0;
	Altitude = 0;
	EcefPositionX = 0;
	EcefPositionY = 0;
	EcefPositionZ = 0;
	VelocityValidity = 0;
	VelocityCoordinateFrame = 0;
	VelocityNorthOrEcefVelocityX = 0;
	VelocityEastOrEcefVelocityY = 0;
	VelocityDownOrEcefVelocityZ = 0;
	LeapSecondValidity = 0;
	ReceiverClockBiasErrorValidity = 0;
	DopDataValidity = 0;
	ConstellationChangeIndicator = 0;
	EstimatedPositionErrorValidity = 0;
	EstimatedVelocityErrorValidity = 0;
	LeapSeconds = 0;
	ReceiverClockBiasError = 0;
	Tdop = 0;
	Vdop = 0;
	Hdop = 0;
	Pdop = 0;
	EstimatedVerticalPositionError = 0;
	EstimatedHorizontalPositionError = 0;
	EstimatedVerticalVelocityError = 0;
	EstimatedHorizontalVelocityError = 0;
	Fom = 0;
	Tfom = 0;
}

bool Msg_1108::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 112) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(RequestAckNak);

	uint32_t bitfieldAtByte20 = 0; // temporary variable holding the bitfield
	bitfieldAtByte20 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte20, 0, TimeReferenceMode, status_ok);
	bitfieldAtByte20 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte20, 1, UnfilteredPointSolution, status_ok);
	bitfieldAtByte20 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte20, 2, 5, GnssMode, status_ok);
	bb.setOffset(20);	bb.put(bitfieldAtByte20);

	bb.setOffset(24);	bb.put(gpsPvtTov);

	uint32_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 0, PositionValidity, status_ok);
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 1, PositionCoordinateFrame, status_ok);
	bb.setOffset(32);	bb.put(bitfieldAtByte32);


	uint32_t bitfieldAtByte52 = 0; // temporary variable holding the bitfield
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte52, 0, VelocityValidity, status_ok);
	bitfieldAtByte52 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte52, 1, VelocityCoordinateFrame, status_ok);
	bb.setOffset(52);	bb.put(bitfieldAtByte52);

	bb.setOffset(56);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(VelocityNorthOrEcefVelocityX / (std::pow(2, -17))));
	bb.setOffset(60);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(VelocityEastOrEcefVelocityY / (std::pow(2, -17))));
	bb.setOffset(64);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(VelocityDownOrEcefVelocityZ / (std::pow(2, -17))));

	uint32_t bitfieldAtByte68 = 0; // temporary variable holding the bitfield
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 0, LeapSecondValidity, status_ok);
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 1, ReceiverClockBiasErrorValidity, status_ok);
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 2, DopDataValidity, status_ok);
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 3, ConstellationChangeIndicator, status_ok);
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 4, EstimatedPositionErrorValidity, status_ok);
	bitfieldAtByte68 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte68, 5, EstimatedVelocityErrorValidity, status_ok);
	bb.setOffset(68);	bb.put(bitfieldAtByte68);

	bb.setOffset(72);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(LeapSeconds / (1.0)));
	bb.setOffset(74);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(ReceiverClockBiasError / (1.0)));

	uint8_t bitfieldAtByte76 = 0; // temporary variable holding the bitfield
	bitfieldAtByte76 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte76, 0, 7, static_cast<uint8_t>CHECK_MAX_UINT8(Tdop / (0.04)), status_ok);
	bb.setOffset(76);	bb.put(bitfieldAtByte76);


	uint8_t bitfieldAtByte77 = 0; // temporary variable holding the bitfield
	bitfieldAtByte77 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte77, 0, 7, static_cast<uint8_t>CHECK_MAX_UINT8(Vdop / (0.04)), status_ok);
	bb.setOffset(77);	bb.put(bitfieldAtByte77);


	uint8_t bitfieldAtByte78 = 0; // temporary variable holding the bitfield
	bitfieldAtByte78 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte78, 0, 7, static_cast<uint8_t>CHECK_MAX_UINT8(Hdop / (0.04)), status_ok);
	bb.setOffset(78);	bb.put(bitfieldAtByte78);


	uint8_t bitfieldAtByte79 = 0; // temporary variable holding the bitfield
	bitfieldAtByte79 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte79, 0, 7, static_cast<uint8_t>CHECK_MAX_UINT8(Pdop / (0.04)), status_ok);
	bb.setOffset(79);	bb.put(bitfieldAtByte79);

	bb.setOffset(80);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(EstimatedVerticalPositionError / (0.01)));
	bb.setOffset(82);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(EstimatedHorizontalPositionError / (0.01)));
	bb.setOffset(84);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(EstimatedVerticalVelocityError / (0.01)));
	bb.setOffset(86);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(EstimatedHorizontalVelocityError / (0.01)));
	bb.setOffset(88);	bb.put(Fom);
	bb.setOffset(92);	bb.put(Tfom);

	if ( PositionCoordinateFrame == 0 ) {
			bb.setOffset(36);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(Latitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(40);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(Longitude / (ECTOS::CONSTANTS::pi*std::pow(2, -31))));
			bb.setOffset(44);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(Altitude / (std::pow(2, -14))));
		}

	if ( PositionCoordinateFrame == 1 ) {
			bb.setOffset(36);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(EcefPositionX / (std::pow(2, -14))));
			bb.setOffset(40);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(EcefPositionY / (std::pow(2, -14))));
			bb.setOffset(44);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(EcefPositionZ / (std::pow(2, -14))));
		}
	Checksum = computeChecksum((uint32_t*)buffer, 28);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1108::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 112) return -2;

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
	bb.setOffset(16);	bb.get(RequestAckNak);

	uint32_t bitfieldAtByte20 = 0; // temporary variable holding the bitfield
	bb.setOffset(20);	bb.get(bitfieldAtByte20);
	TimeReferenceMode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte20, 0,  status_ok);
	UnfilteredPointSolution = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte20, 1,  status_ok);
	GnssMode = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte20, 2, 5,  status_ok);
	bb.setOffset(24);	bb.get(gpsPvtTov);

	uint32_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bb.setOffset(32);	bb.get(bitfieldAtByte32);
	PositionValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 0,  status_ok);
	PositionCoordinateFrame = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 1,  status_ok);

	uint32_t bitfieldAtByte52 = 0; // temporary variable holding the bitfield
	bb.setOffset(52);	bb.get(bitfieldAtByte52);
	VelocityValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte52, 0,  status_ok);
	VelocityCoordinateFrame = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte52, 1,  status_ok);
	bb.setOffset(56);	VelocityNorthOrEcefVelocityX = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -17));
	bb.setOffset(60);	VelocityEastOrEcefVelocityY = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -17));
	bb.setOffset(64);	VelocityDownOrEcefVelocityZ = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -17));

	uint32_t bitfieldAtByte68 = 0; // temporary variable holding the bitfield
	bb.setOffset(68);	bb.get(bitfieldAtByte68);
	LeapSecondValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 0,  status_ok);
	ReceiverClockBiasErrorValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 1,  status_ok);
	DopDataValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 2,  status_ok);
	ConstellationChangeIndicator = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 3,  status_ok);
	EstimatedPositionErrorValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 4,  status_ok);
	EstimatedVelocityErrorValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte68, 5,  status_ok);
	bb.setOffset(72);	LeapSeconds = static_cast<float>(bb.get<uint16_t>()) * (1.0);
	bb.setOffset(74);	ReceiverClockBiasError = static_cast<float>(bb.get<uint16_t>()) * (1.0);

	uint8_t bitfieldAtByte76 = 0; // temporary variable holding the bitfield
	bb.setOffset(76);	bb.get(bitfieldAtByte76);
	Tdop = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte76, 0, 7,  status_ok);

	uint8_t bitfieldAtByte77 = 0; // temporary variable holding the bitfield
	bb.setOffset(77);	bb.get(bitfieldAtByte77);
	Vdop = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte77, 0, 7,  status_ok);

	uint8_t bitfieldAtByte78 = 0; // temporary variable holding the bitfield
	bb.setOffset(78);	bb.get(bitfieldAtByte78);
	Hdop = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte78, 0, 7,  status_ok);

	uint8_t bitfieldAtByte79 = 0; // temporary variable holding the bitfield
	bb.setOffset(79);	bb.get(bitfieldAtByte79);
	Pdop = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte79, 0, 7,  status_ok);
	bb.setOffset(80);	EstimatedVerticalPositionError = static_cast<float>(bb.get<uint16_t>()) * (0.01);
	bb.setOffset(82);	EstimatedHorizontalPositionError = static_cast<float>(bb.get<uint16_t>()) * (0.01);
	bb.setOffset(84);	EstimatedVerticalVelocityError = static_cast<float>(bb.get<uint16_t>()) * (0.01);
	bb.setOffset(86);	EstimatedHorizontalVelocityError = static_cast<float>(bb.get<uint16_t>()) * (0.01);
	bb.setOffset(88);	bb.get(Fom);
	bb.setOffset(92);	bb.get(Tfom);

	if ( PositionCoordinateFrame == 0 ) {
		 	bb.setOffset(36);	Latitude = static_cast<double>(bb.get<uint32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(40);	Longitude = static_cast<double>(bb.get<uint32_t>()) * (ECTOS::CONSTANTS::pi*std::pow(2, -31));
		 	bb.setOffset(44);	Altitude = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -14));
}

	if ( PositionCoordinateFrame == 1 ) {
		 	bb.setOffset(36);	EcefPositionX = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -14));
		 	bb.setOffset(40);	EcefPositionY = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -14));
		 	bb.setOffset(44);	EcefPositionZ = static_cast<double>(bb.get<uint32_t>()) * (std::pow(2, -14));
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 28);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

