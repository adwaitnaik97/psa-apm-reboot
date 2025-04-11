#include <include/HGuideAPI.h>
#include <include/Msg_6723.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6723::AddressId;
const uint32_t Msg_6723::MessageId;
const uint32_t Msg_6723::MessageLength;

Msg_6723::Msg_6723()
{
	Default();
}

void Msg_6723::Default()
{
	Checksum = 0;
	systemTov = 0;
	version = 0;
	serialNumber = 0;
	offsetOfData = 0;
	profileConfiguration.Default();
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	minute = 0;
	seconds = 0;
	microseconds = 0;
	speedOfSounds = 0;
	temperature = 0;
	pressure = 0;
	heading = 0;
	pitch = 0;
	roll = 0;
	numCells = 0;
	coorinateSystem = 0;
	numBeams = 0;
	cellSize = 0;
	blanking = 0;
	nomCorrelation = 0;
	tempPressureSensor = 0;
	batteryVoltage = 0;
	magX = 0;
	magY = 0;
	magZ = 0;
	accelX = 0;
	accelY = 0;
	accelZ = 0;
	ambiguityVelocity = 0;
	dataSet1_beam = 0;
	dataSet2_beam = 0;
	dataSet3_beam = 0;
	dataSet4_beam = 0;
	transmitEnergy = 0;
	velocityScaling = 0;
	powerLevel = 0;
	tempMag = 0;
	tempRtc = 0;
	error.Default();
	cpu_load = 0;
	status0_used = 0;
	status.Default();
	ensembleCounter = 0;
}

bool Msg_6723::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 100) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(version);
	bb.setOffset(24);	bb.put(serialNumber);
	bb.setOffset(25);	bb.put(offsetOfData);

	uint16_t profileConfiguration_tmp = 0; // temporary variable holding the custom bitfield
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 0, profileConfiguration.pressureValid, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 1, profileConfiguration.temperatureValid, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 2, profileConfiguration.compassValid, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 3, profileConfiguration.tiltValid, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 5, profileConfiguration.velocityIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 6, profileConfiguration.amplitudeIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 7, profileConfiguration.correlationIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 8, profileConfiguration.altimeterIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 9, profileConfiguration.altimeterRawIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 10, profileConfiguration.astIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 11, profileConfiguration.echoSounderIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 12, profileConfiguration.ahrsIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 13, profileConfiguration.percentageGoodIncluded, status_ok);
	profileConfiguration_tmp = ECTOS::BIT_UTILITIES::PackBool(profileConfiguration_tmp, 14, profileConfiguration.stdvIncluded, status_ok);
	bb.setOffset(26);	bb.put(profileConfiguration_tmp);

	bb.setOffset(30);	bb.put(year);
	bb.setOffset(31);	bb.put(month);
	bb.setOffset(32);	bb.put(day);
	bb.setOffset(33);	bb.put(hour);
	bb.setOffset(34);	bb.put(minute);
	bb.setOffset(35);	bb.put(seconds);
	bb.setOffset(36);	bb.put(microseconds / (100));
	bb.setOffset(40);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(speedOfSounds / (0.1)));
	bb.setOffset(42);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(temperature / (0.01)));
	bb.setOffset(44);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(pressure / (0.001)));
	bb.setOffset(48);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(heading / (0.01)));
	bb.setOffset(50);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(pitch / (0.01)));
	bb.setOffset(52);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(roll / (0.01)));

	uint16_t bitfieldAtByte54 = 0; // temporary variable holding the bitfield
	bitfieldAtByte54 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte54, 0, 9, numCells, status_ok);
	bitfieldAtByte54 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte54, 10, 11, coorinateSystem, status_ok);
	bitfieldAtByte54 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte54, 12, 15, numBeams, status_ok);
	bb.setOffset(54);	bb.put(bitfieldAtByte54);

	bb.setOffset(56);	bb.put(cellSize);
	bb.setOffset(58);	bb.put(blanking);
	bb.setOffset(60);	bb.put(nomCorrelation);
	bb.setOffset(61);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(tempPressureSensor / (0.2)));
	bb.setOffset(62);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(batteryVoltage / (0.1)));
	bb.setOffset(64);	bb.put(magX);
	bb.setOffset(66);	bb.put(magY);
	bb.setOffset(68);	bb.put(magZ);
	bb.setOffset(70);	bb.put(accelX);
	bb.setOffset(72);	bb.put(accelY);
	bb.setOffset(74);	bb.put(accelZ);
	bb.setOffset(76);	bb.put(ambiguityVelocity);

	uint8_t bitfieldAtByte78 = 0; // temporary variable holding the bitfield
	bitfieldAtByte78 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte78, 0, 3, dataSet1_beam, status_ok);
	bitfieldAtByte78 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte78, 4, 7, dataSet2_beam, status_ok);
	bb.setOffset(78);	bb.put(bitfieldAtByte78);


	uint8_t bitfieldAtByte79 = 0; // temporary variable holding the bitfield
	bitfieldAtByte79 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte79, 0, 3, dataSet3_beam, status_ok);
	bitfieldAtByte79 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte79, 4, 7, dataSet4_beam, status_ok);
	bb.setOffset(79);	bb.put(bitfieldAtByte79);

	bb.setOffset(80);	bb.put(transmitEnergy);
	bb.setOffset(82);	bb.put(velocityScaling);
	bb.setOffset(83);	bb.put(powerLevel);
	bb.setOffset(84);	bb.put(tempMag);
	bb.setOffset(86);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(tempRtc / (0.01)));

	uint16_t error_tmp = 0; // temporary variable holding the custom bitfield
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 0, error.data_retrieval_fifo, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 1, error.data_retrieval_overflow, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 2, error.data_retrieval_overflow2, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 3, error.data_retrieval_underrun, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 4, error.data_retrieval_missing_samples, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 5, error.sensor_read_failure, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 8, error.beam0_in_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 9, error.beam0_quad_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 10, error.beam1_in_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 11, error.beam1_quad_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 12, error.beam2_in_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 13, error.beam2_quad_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 14, error.beam3_in_phase_tag_error, status_ok);
	error_tmp = ECTOS::BIT_UTILITIES::PackBool(error_tmp, 15, error.beam3_quad_phase_tag_error, status_ok);
	bb.setOffset(88);	bb.put(error_tmp);


	uint16_t bitfieldAtByte90 = 0; // temporary variable holding the bitfield
	bitfieldAtByte90 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte90, 15, status0_used, status_ok);
	bb.setOffset(90);	bb.put(bitfieldAtByte90);


	uint32_t status_tmp = 0; // temporary variable holding the custom bitfield
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 1, status.bdScaling, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 5, 8, status.echoFrequency, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 11, status.boostRunning, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 12, status.telemetryData, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 13, 15, status.echoIndex, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 16, status.configurationActive, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::PackBool(status_tmp, 17, status.lastMeasVoltage, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 18, 21, status.prevWakeup, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 22, 24, status.orientationAuto, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 25, 27, status.orientation, status_ok);
	status_tmp = ECTOS::BIT_UTILITIES::Pack(status_tmp, 28, 31, status.wakeup, status_ok);
	bb.setOffset(92);	bb.put(status_tmp);

	bb.setOffset(96);	bb.put(ensembleCounter);

	if ( status0_used == 1 ) {
			uint16_t bitfieldAtByte90 = 0; // temporary variable holding the bitfield
		bitfieldAtByte90 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte90, 0, 2, cpu_load, status_ok);
		bb.setOffset(90);	bb.put(bitfieldAtByte90);
	
		}
	Checksum = computeChecksum((uint32_t*)buffer, 25);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6723::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 100) return -2;

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
	bb.setOffset(24);	bb.get(version);
	bb.setOffset(24);	bb.get(serialNumber);
	bb.setOffset(25);	bb.get(offsetOfData);

	uint16_t profileConfiguration_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(26);	bb.get(profileConfiguration_tmp);
	profileConfiguration.pressureValid = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 0,  status_ok);
	profileConfiguration.temperatureValid = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 1,  status_ok);
	profileConfiguration.compassValid = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 2,  status_ok);
	profileConfiguration.tiltValid = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 3,  status_ok);
	profileConfiguration.velocityIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 5,  status_ok);
	profileConfiguration.amplitudeIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 6,  status_ok);
	profileConfiguration.correlationIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 7,  status_ok);
	profileConfiguration.altimeterIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 8,  status_ok);
	profileConfiguration.altimeterRawIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 9,  status_ok);
	profileConfiguration.astIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 10,  status_ok);
	profileConfiguration.echoSounderIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 11,  status_ok);
	profileConfiguration.ahrsIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 12,  status_ok);
	profileConfiguration.percentageGoodIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 13,  status_ok);
	profileConfiguration.stdvIncluded = ECTOS::BIT_UTILITIES::UnPackBool(profileConfiguration_tmp, 14,  status_ok);
	bb.setOffset(30);	bb.get(year);
	bb.setOffset(31);	bb.get(month);
	bb.setOffset(32);	bb.get(day);
	bb.setOffset(33);	bb.get(hour);
	bb.setOffset(34);	bb.get(minute);
	bb.setOffset(35);	bb.get(seconds);
	bb.setOffset(36);	microseconds = bb.get<uint16_t>() * (100);
	bb.setOffset(40);	speedOfSounds = static_cast<float>(bb.get<uint16_t>()) * (0.1);
	bb.setOffset(42);	temperature = static_cast<float>(bb.get<int16_t>()) * (0.01);
	bb.setOffset(44);	pressure = static_cast<float>(bb.get<uint32_t>()) * (0.001);
	bb.setOffset(48);	heading = static_cast<float>(bb.get<uint16_t>()) * (0.01);
	bb.setOffset(50);	pitch = static_cast<float>(bb.get<int16_t>()) * (0.01);
	bb.setOffset(52);	roll = static_cast<float>(bb.get<int16_t>()) * (0.01);

	uint16_t bitfieldAtByte54 = 0; // temporary variable holding the bitfield
	bb.setOffset(54);	bb.get(bitfieldAtByte54);
	numCells = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte54, 0, 9,  status_ok);
	coorinateSystem = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte54, 10, 11,  status_ok);
	numBeams = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte54, 12, 15,  status_ok);
	bb.setOffset(56);	bb.get(cellSize);
	bb.setOffset(58);	bb.get(blanking);
	bb.setOffset(60);	bb.get(nomCorrelation);
	bb.setOffset(61);	tempPressureSensor = static_cast<float>(bb.get<uint8_t>()) * (0.2);
	bb.setOffset(62);	batteryVoltage = static_cast<float>(bb.get<uint16_t>()) * (0.1);
	bb.setOffset(64);	bb.get(magX);
	bb.setOffset(66);	bb.get(magY);
	bb.setOffset(68);	bb.get(magZ);
	bb.setOffset(70);	bb.get(accelX);
	bb.setOffset(72);	bb.get(accelY);
	bb.setOffset(74);	bb.get(accelZ);
	bb.setOffset(76);	bb.get(ambiguityVelocity);

	uint8_t bitfieldAtByte78 = 0; // temporary variable holding the bitfield
	bb.setOffset(78);	bb.get(bitfieldAtByte78);
	dataSet1_beam = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte78, 0, 3,  status_ok);
	dataSet2_beam = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte78, 4, 7,  status_ok);

	uint8_t bitfieldAtByte79 = 0; // temporary variable holding the bitfield
	bb.setOffset(79);	bb.get(bitfieldAtByte79);
	dataSet3_beam = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte79, 0, 3,  status_ok);
	dataSet4_beam = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte79, 4, 7,  status_ok);
	bb.setOffset(80);	bb.get(transmitEnergy);
	bb.setOffset(82);	bb.get(velocityScaling);
	bb.setOffset(83);	bb.get(powerLevel);
	bb.setOffset(84);	bb.get(tempMag);
	bb.setOffset(86);	tempRtc = static_cast<float>(bb.get<int16_t>()) * (0.01);

	uint16_t error_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(88);	bb.get(error_tmp);
	error.data_retrieval_fifo = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 0,  status_ok);
	error.data_retrieval_overflow = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 1,  status_ok);
	error.data_retrieval_overflow2 = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 2,  status_ok);
	error.data_retrieval_underrun = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 3,  status_ok);
	error.data_retrieval_missing_samples = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 4,  status_ok);
	error.sensor_read_failure = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 5,  status_ok);
	error.beam0_in_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 8,  status_ok);
	error.beam0_quad_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 9,  status_ok);
	error.beam1_in_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 10,  status_ok);
	error.beam1_quad_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 11,  status_ok);
	error.beam2_in_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 12,  status_ok);
	error.beam2_quad_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 13,  status_ok);
	error.beam3_in_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 14,  status_ok);
	error.beam3_quad_phase_tag_error = ECTOS::BIT_UTILITIES::UnPackBool(error_tmp, 15,  status_ok);

	uint16_t bitfieldAtByte90 = 0; // temporary variable holding the bitfield
	bb.setOffset(90);	bb.get(bitfieldAtByte90);
	status0_used = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte90, 15,  status_ok);

	uint32_t status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(92);	bb.get(status_tmp);
	status.bdScaling = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 1,  status_ok);
	status.echoFrequency = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 5, 8,  status_ok);
	status.boostRunning = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 11,  status_ok);
	status.telemetryData = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 12,  status_ok);
	status.echoIndex = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 13, 15,  status_ok);
	status.configurationActive = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 16,  status_ok);
	status.lastMeasVoltage = ECTOS::BIT_UTILITIES::UnPackBool(status_tmp, 17,  status_ok);
	status.prevWakeup = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 18, 21,  status_ok);
	status.orientationAuto = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 22, 24,  status_ok);
	status.orientation = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 25, 27,  status_ok);
	status.wakeup = ECTOS::BIT_UTILITIES::UnPack(status_tmp, 28, 31,  status_ok);
	bb.setOffset(96);	bb.get(ensembleCounter);

	if ( status0_used == 1 ) {
		uint16_t bitfieldAtByte90 = 0; // temporary variable holding the bitfield
		bb.setOffset(90);	bb.get(bitfieldAtByte90);
		cpu_load = ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte90, 0, 2,  status_ok);
	}
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 25);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

