#include <include/HGuideAPI.h>
#include <include/Msg_4438.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_4438::AddressId;
const uint32_t Msg_4438::MessageId;
const uint32_t Msg_4438::MessageLength;

Msg_4438::Msg_4438()
{
	Default();
}

void Msg_4438::Default()
{
	Checksum = 0;
	QdecEnable = 0;
	OdometerLeverArmX = 0;
	OdometerLeverArmY = 0;
	OdometerLeverArmZ = 0;
	OdometerTheshold = 1;
	ChangeOdometerLeverArm = 0;
	QdecDistancePerPulse = 0.0001578;
	OdometerMeasurementNoise = 0.25;
	OdometerInitialScaleFactorUncertainty = 0.1;
	OdometerScaleFactorNoise = 1E-05;
	OdometerYawBoresightStdv = 0.07;
	OdometerYawBoresightProcessNoise = 2E-06;
	OdometerInitialPitchBoresightUncertainty = 0.07;
	OdometerInitialPitchBoresightProcessNoise = 2E-06;
}

bool Msg_4438::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 96) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t bitfieldAtByte24 = 0; // temporary variable holding the bitfield
	bitfieldAtByte24 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte24, 0, QdecEnable, status_ok);
	bb.setOffset(24);	bb.put(bitfieldAtByte24);

	bb.setOffset(28);	bb.put(QdecDistancePerPulse);
	bb.setOffset(32);	bb.put(OdometerLeverArmX);
	bb.setOffset(36);	bb.put(OdometerLeverArmY);
	bb.setOffset(40);	bb.put(OdometerLeverArmZ);
	bb.setOffset(44);	bb.put(OdometerMeasurementNoise);
	bb.setOffset(48);	bb.put(OdometerTheshold);
	bb.setOffset(52);	bb.put(OdometerInitialScaleFactorUncertainty);
	bb.setOffset(56);	bb.put(OdometerScaleFactorNoise);
	bb.setOffset(60);	bb.put(OdometerYawBoresightStdv);
	bb.setOffset(64);	bb.put(OdometerYawBoresightProcessNoise);
	bb.setOffset(68);	bb.put(OdometerInitialPitchBoresightUncertainty);
	bb.setOffset(72);	bb.put(OdometerInitialPitchBoresightProcessNoise);

	uint32_t bitfieldAtByte76 = 0; // temporary variable holding the bitfield
	bitfieldAtByte76 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte76, 0, ChangeOdometerLeverArm, status_ok);
	bb.setOffset(76);	bb.put(bitfieldAtByte76);

	Checksum = computeChecksum((uint32_t*)buffer, 24);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_4438::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 96) return -2;

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

	uint32_t bitfieldAtByte24 = 0; // temporary variable holding the bitfield
	bb.setOffset(24);	bb.get(bitfieldAtByte24);
	QdecEnable = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte24, 0,  status_ok);
	bb.setOffset(28);	bb.get(QdecDistancePerPulse);
	bb.setOffset(32);	bb.get(OdometerLeverArmX);
	bb.setOffset(36);	bb.get(OdometerLeverArmY);
	bb.setOffset(40);	bb.get(OdometerLeverArmZ);
	bb.setOffset(44);	bb.get(OdometerMeasurementNoise);
	bb.setOffset(48);	bb.get(OdometerTheshold);
	bb.setOffset(52);	bb.get(OdometerInitialScaleFactorUncertainty);
	bb.setOffset(56);	bb.get(OdometerScaleFactorNoise);
	bb.setOffset(60);	bb.get(OdometerYawBoresightStdv);
	bb.setOffset(64);	bb.get(OdometerYawBoresightProcessNoise);
	bb.setOffset(68);	bb.get(OdometerInitialPitchBoresightUncertainty);
	bb.setOffset(72);	bb.get(OdometerInitialPitchBoresightProcessNoise);

	uint32_t bitfieldAtByte76 = 0; // temporary variable holding the bitfield
	bb.setOffset(76);	bb.get(bitfieldAtByte76);
	ChangeOdometerLeverArm = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte76, 0,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 24);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

