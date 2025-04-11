#include <include/HGuideAPI.h>
#include <include/Msg_4401.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4401::AddressId;
const uint32_t Msg_4401::MessageId;
const uint32_t Msg_4401::MessageLength;

Msg_4401::Msg_4401()
{
	Default();
}

void Msg_4401::Default()
{
	Checksum = 0;
	RequestACKNAKReply = 1;
	TOV_Mode = 0;
	input_coordinate_frame = static_cast<coordinate_frame_t>(VEHICLE_BODY);
	solutionTov = 0;
	PositionValidity = 0;
	Latitude = 0;
	Longitude = 0;
	Altitude = 0;
	VelocityValidity = 0;
	NorthVelocity = 0;
	EastVelocity = 0;
	DownVelocity = 0;
	AttitudeValidity = 0;
	Roll = 0;
	Pitch = 0;
	TrueHeading = 0;
	PositionStdvNorth = 100.0;
	PositionStdvEast = 100.0;
	PositionStdvDown = 100.0;
	VelocityStdvNorth = 1.0;
	VelocityStdvEast = 1.0;
	VelocityStdvDown = 1.0;
	AttitudeStdvRoll = 0.0873;
	AttitudeStdvPitch = 0.0873;
	AttitudeStdvTrueHeading = 0.0873;
}

bool Msg_4401::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 160) return false;

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


	uint32_t bitfieldAtByte17 = 0; // temporary variable holding the bitfield
	bitfieldAtByte17 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte17, 0, TOV_Mode, status_ok);
	bb.setOffset(17);	bb.put(bitfieldAtByte17);

	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(input_coordinate_frame));
	bb.setOffset(24);	bb.put(solutionTov);

	uint32_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bitfieldAtByte32 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte32, 0, PositionValidity, status_ok);
	bb.setOffset(32);	bb.put(bitfieldAtByte32);


	uint32_t bitfieldAtByte60 = 0; // temporary variable holding the bitfield
	bitfieldAtByte60 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte60, 0, VelocityValidity, status_ok);
	bb.setOffset(60);	bb.put(bitfieldAtByte60);


	uint32_t bitfieldAtByte88 = 0; // temporary variable holding the bitfield
	bitfieldAtByte88 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte88, 0, AttitudeValidity, status_ok);
	bb.setOffset(88);	bb.put(bitfieldAtByte88);


	if ( PositionValidity == 1 ) {
			bb.setOffset(36);	bb.put(Latitude);
			bb.setOffset(44);	bb.put(Longitude);
			bb.setOffset(52);	bb.put(Altitude);
			bb.setOffset(120);	bb.put(PositionStdvNorth);
			bb.setOffset(124);	bb.put(PositionStdvEast);
			bb.setOffset(128);	bb.put(PositionStdvDown);
		}

	if ( VelocityValidity == 1 ) {
			bb.setOffset(64);	bb.put(NorthVelocity);
			bb.setOffset(72);	bb.put(EastVelocity);
			bb.setOffset(80);	bb.put(DownVelocity);
			bb.setOffset(132);	bb.put(VelocityStdvNorth);
			bb.setOffset(136);	bb.put(VelocityStdvEast);
			bb.setOffset(140);	bb.put(VelocityStdvDown);
		}

	if ( AttitudeValidity == 1 ) {
			bb.setOffset(92);	bb.put(Roll);
			bb.setOffset(100);	bb.put(Pitch);
			bb.setOffset(108);	bb.put(TrueHeading);
			bb.setOffset(144);	bb.put(AttitudeStdvRoll);
			bb.setOffset(148);	bb.put(AttitudeStdvPitch);
			bb.setOffset(152);	bb.put(AttitudeStdvTrueHeading);
		}
	Checksum = computeChecksum((uint32_t*)buffer, 40);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4401::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 160) return -2;

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

	uint32_t bitfieldAtByte17 = 0; // temporary variable holding the bitfield
	bb.setOffset(17);	bb.get(bitfieldAtByte17);
	TOV_Mode = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte17, 0,  status_ok);
	bb.setOffset(20);	input_coordinate_frame = static_cast<coordinate_frame_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(solutionTov);

	uint32_t bitfieldAtByte32 = 0; // temporary variable holding the bitfield
	bb.setOffset(32);	bb.get(bitfieldAtByte32);
	PositionValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte32, 0,  status_ok);

	uint32_t bitfieldAtByte60 = 0; // temporary variable holding the bitfield
	bb.setOffset(60);	bb.get(bitfieldAtByte60);
	VelocityValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte60, 0,  status_ok);

	uint32_t bitfieldAtByte88 = 0; // temporary variable holding the bitfield
	bb.setOffset(88);	bb.get(bitfieldAtByte88);
	AttitudeValidity = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte88, 0,  status_ok);

	if ( PositionValidity == 1 ) {
		 	bb.setOffset(36);	bb.get(Latitude);
		 	bb.setOffset(44);	bb.get(Longitude);
		 	bb.setOffset(52);	bb.get(Altitude);
		 	bb.setOffset(120);	bb.get(PositionStdvNorth);
		 	bb.setOffset(124);	bb.get(PositionStdvEast);
		 	bb.setOffset(128);	bb.get(PositionStdvDown);
}

	if ( VelocityValidity == 1 ) {
		 	bb.setOffset(64);	bb.get(NorthVelocity);
		 	bb.setOffset(72);	bb.get(EastVelocity);
		 	bb.setOffset(80);	bb.get(DownVelocity);
		 	bb.setOffset(132);	bb.get(VelocityStdvNorth);
		 	bb.setOffset(136);	bb.get(VelocityStdvEast);
		 	bb.setOffset(140);	bb.get(VelocityStdvDown);
}

	if ( AttitudeValidity == 1 ) {
		 	bb.setOffset(92);	bb.get(Roll);
		 	bb.setOffset(100);	bb.get(Pitch);
		 	bb.setOffset(108);	bb.get(TrueHeading);
		 	bb.setOffset(144);	bb.get(AttitudeStdvRoll);
		 	bb.setOffset(148);	bb.get(AttitudeStdvPitch);
		 	bb.setOffset(152);	bb.get(AttitudeStdvTrueHeading);
}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 40);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

