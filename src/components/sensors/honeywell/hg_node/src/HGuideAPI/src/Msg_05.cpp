#include <include/HGuideAPI.h>
#include <include/Msg_05.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_05::SyncByte;
const uint8_t Msg_05::MessageID;

Msg_05::Msg_05()
{
	Default();
}

void Msg_05::Default()
{
	AngularRateX = 0;
	AngularRateY = 0;
	AngularRateZ = 0;
	LinearAccelerationX = 0;
	LinearAccelerationY = 0;
	LinearAccelerationZ = 0;
	MagneticFieldX = 0;
	MagneticFieldY = 0;
	MagneticFieldZ = 0;
	MainStatusWord.Default();
	MultiPlexedStatusWord.Default();
	DeltaAngleX = 0;
	DeltaAngleY = 0;
	DeltaAngleZ = 0;
	DeltaVelocityX = 0;
	DeltaVelocityY = 0;
	DeltaVelocityZ = 0;
	Checksum = 0;
}

bool Msg_05::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 50) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateX / (0.00114440918)));
	bb.setOffset(4);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateY / (0.00114440918)));
	bb.setOffset(6);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateZ / (0.00114440918)));
	bb.setOffset(8);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationX / (0.02232422)));
	bb.setOffset(10);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationY / (0.02232422)));
	bb.setOffset(12);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationZ / (0.02232422)));
	bb.setOffset(14);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(MagneticFieldX / (0.438404)));
	bb.setOffset(16);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(MagneticFieldY / (0.438404)));
	bb.setOffset(18);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(MagneticFieldZ / (0.438404)));

	uint16_t MainStatusWord_tmp = 0; // temporary variable holding the custom bitfield
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::Pack(MainStatusWord_tmp, 0, 3, MainStatusWord.MuxStatusCounter, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 4, MainStatusWord.IMU_OK, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 5, MainStatusWord.SensorBoardInit, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 6, MainStatusWord.AccelXValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 7, MainStatusWord.AccelYValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 8, MainStatusWord.AccelZValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 9, MainStatusWord.GyroXValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 10, MainStatusWord.GyroYValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 11, MainStatusWord.GyroZValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 12, MainStatusWord.MagnetometerValidity, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 13, MainStatusWord.PowerUp_BIT, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 14, MainStatusWord.Continuous_BIT, status_ok);
	MainStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MainStatusWord_tmp, 15, MainStatusWord.PowerUp_test, status_ok);
	bb.setOffset(20);	bb.put(MainStatusWord_tmp);


	uint16_t MultiPlexedStatusWord_tmp = 0; // temporary variable holding the custom bitfield

	if ( MainStatusWord.MuxStatusCounter == 0 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord_tmp, 0, 15, MultiPlexedStatusWord.EmbeddedSoftwareVersion, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 1 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 0, MultiPlexedStatusWord.SensorElectronicsStatus, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 1, MultiPlexedStatusWord.SensorDataReadyStatus, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.TemperatureStatus, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 3, MultiPlexedStatusWord.AccelerometerXHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 4, MultiPlexedStatusWord.AccelerometerYHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 5, MultiPlexedStatusWord.AccelerometerZHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 6, MultiPlexedStatusWord.GyroXHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 7, MultiPlexedStatusWord.GyroYHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 8, MultiPlexedStatusWord.GyroZHealth, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 2 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.TemperatureStatusLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 3, MultiPlexedStatusWord.AccelerometerXHealthLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 4, MultiPlexedStatusWord.AccelerometerYHealthLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 5, MultiPlexedStatusWord.AccelerometerZHealthLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 6, MultiPlexedStatusWord.GyroXHealthLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 7, MultiPlexedStatusWord.GyroYHealthLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 8, MultiPlexedStatusWord.GyroZHealthLatched, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 3 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 0, MultiPlexedStatusWord.MagnetometerXHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 1, MultiPlexedStatusWord.MagnetometerYHealth, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.MagnetometerZHealth, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 5 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 0, MultiPlexedStatusWord.LoopCompletionTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 1, MultiPlexedStatusWord.RAMTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.CoefficientTableCRCTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 3, MultiPlexedStatusWord.ConfigurationTableCRCTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 4, MultiPlexedStatusWord.NormalModeSWCRCTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 6, MultiPlexedStatusWord.StackOverflowTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 7, MultiPlexedStatusWord.WatchdogTimerTest, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 8, MultiPlexedStatusWord.ProcessorTest, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 6 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 0, MultiPlexedStatusWord.LoopCompletionTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 1, MultiPlexedStatusWord.RAMTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.CoefficientTableCRCTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 3, MultiPlexedStatusWord.ConfigurationTableCRCTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 4, MultiPlexedStatusWord.NormalModeSWCRCTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 6, MultiPlexedStatusWord.StackOverflowTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 7, MultiPlexedStatusWord.WatchdogTimerTestLatched, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 8, MultiPlexedStatusWord.ProcessorTestLatched, status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 7 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord_tmp, 0, 15, static_cast<int16_t>CHECK_MAX_INT16(MultiPlexedStatusWord.SensorTemperature / (0.0039)), status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 8 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord_tmp, 0, 15, static_cast<int16_t>CHECK_MAX_INT16(MultiPlexedStatusWord.MagnetometerTemperature / (0.0039)), status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 9 ) {
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 0, MultiPlexedStatusWord.DIO1, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 1, MultiPlexedStatusWord.DIO2, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 2, MultiPlexedStatusWord.DIO3, status_ok);
		MultiPlexedStatusWord_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord_tmp, 3, MultiPlexedStatusWord.DIO4, status_ok);
	}
	bb.setOffset(22);	bb.put(MultiPlexedStatusWord_tmp);

	bb.setOffset(24);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleX / (5.820766E-11)));
	bb.setOffset(28);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleY / (5.820766E-11)));
	bb.setOffset(32);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleZ / (5.820766E-11)));
	bb.setOffset(36);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityX / (1.135468E-09)));
	bb.setOffset(40);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityY / (1.135468E-09)));
	bb.setOffset(44);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityZ / (1.135468E-09)));
	int cIndex = 48;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 50);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_05::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 50) return -2;

	ECTOS::BYTE_BUFFER::ByteInputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	int constCheck = 0;
	int numConsts = 0;
	// Compare with the static const variable to ensure they match. 
	uint8_t SyncByte_In;
	bb.setOffset(0);	bb.get(SyncByte_In);
	constCheck |= (SyncByte != SyncByte_In) << numConsts++;
	// Compare with the static const variable to ensure they match. 
	uint8_t MessageID_In;
	bb.setOffset(1);	bb.get(MessageID_In);
	constCheck |= (MessageID != MessageID_In) << numConsts++;
	bb.setOffset(2);	AngularRateX = static_cast<float>(bb.get<int16_t>()) * (0.00114440918);
	bb.setOffset(4);	AngularRateY = static_cast<float>(bb.get<int16_t>()) * (0.00114440918);
	bb.setOffset(6);	AngularRateZ = static_cast<float>(bb.get<int16_t>()) * (0.00114440918);
	bb.setOffset(8);	LinearAccelerationX = static_cast<float>(bb.get<int16_t>()) * (0.02232422);
	bb.setOffset(10);	LinearAccelerationY = static_cast<float>(bb.get<int16_t>()) * (0.02232422);
	bb.setOffset(12);	LinearAccelerationZ = static_cast<float>(bb.get<int16_t>()) * (0.02232422);
	bb.setOffset(14);	MagneticFieldX = static_cast<float>(bb.get<int16_t>()) * (0.438404);
	bb.setOffset(16);	MagneticFieldY = static_cast<float>(bb.get<int16_t>()) * (0.438404);
	bb.setOffset(18);	MagneticFieldZ = static_cast<float>(bb.get<int16_t>()) * (0.438404);

	uint16_t MainStatusWord_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(20);	bb.get(MainStatusWord_tmp);
	MainStatusWord.MuxStatusCounter = ECTOS::BIT_UTILITIES::UnPack(MainStatusWord_tmp, 0, 3,  status_ok);
	MainStatusWord.IMU_OK = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 4,  status_ok);
	MainStatusWord.SensorBoardInit = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 5,  status_ok);
	MainStatusWord.AccelXValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 6,  status_ok);
	MainStatusWord.AccelYValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 7,  status_ok);
	MainStatusWord.AccelZValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 8,  status_ok);
	MainStatusWord.GyroXValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 9,  status_ok);
	MainStatusWord.GyroYValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 10,  status_ok);
	MainStatusWord.GyroZValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 11,  status_ok);
	MainStatusWord.MagnetometerValidity = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 12,  status_ok);
	MainStatusWord.PowerUp_BIT = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 13,  status_ok);
	MainStatusWord.Continuous_BIT = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 14,  status_ok);
	MainStatusWord.PowerUp_test = ECTOS::BIT_UTILITIES::UnPackBool(MainStatusWord_tmp, 15,  status_ok);

	uint16_t MultiPlexedStatusWord_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(22);	bb.get(MultiPlexedStatusWord_tmp);

	if ( MainStatusWord.MuxStatusCounter == 0 ) {
		MultiPlexedStatusWord.EmbeddedSoftwareVersion = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord_tmp, 0, 15,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 1 ) {
		MultiPlexedStatusWord.SensorElectronicsStatus = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 0,  status_ok);
		MultiPlexedStatusWord.SensorDataReadyStatus = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 1,  status_ok);
		MultiPlexedStatusWord.TemperatureStatus = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
		MultiPlexedStatusWord.AccelerometerXHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 3,  status_ok);
		MultiPlexedStatusWord.AccelerometerYHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 4,  status_ok);
		MultiPlexedStatusWord.AccelerometerZHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 5,  status_ok);
		MultiPlexedStatusWord.GyroXHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 6,  status_ok);
		MultiPlexedStatusWord.GyroYHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 7,  status_ok);
		MultiPlexedStatusWord.GyroZHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 8,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 2 ) {
		MultiPlexedStatusWord.TemperatureStatusLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
		MultiPlexedStatusWord.AccelerometerXHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 3,  status_ok);
		MultiPlexedStatusWord.AccelerometerYHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 4,  status_ok);
		MultiPlexedStatusWord.AccelerometerZHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 5,  status_ok);
		MultiPlexedStatusWord.GyroXHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 6,  status_ok);
		MultiPlexedStatusWord.GyroYHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 7,  status_ok);
		MultiPlexedStatusWord.GyroZHealthLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 8,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 3 ) {
		MultiPlexedStatusWord.MagnetometerXHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 0,  status_ok);
		MultiPlexedStatusWord.MagnetometerYHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 1,  status_ok);
		MultiPlexedStatusWord.MagnetometerZHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 5 ) {
		MultiPlexedStatusWord.LoopCompletionTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 0,  status_ok);
		MultiPlexedStatusWord.RAMTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 1,  status_ok);
		MultiPlexedStatusWord.CoefficientTableCRCTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
		MultiPlexedStatusWord.ConfigurationTableCRCTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 3,  status_ok);
		MultiPlexedStatusWord.NormalModeSWCRCTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 4,  status_ok);
		MultiPlexedStatusWord.StackOverflowTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 6,  status_ok);
		MultiPlexedStatusWord.WatchdogTimerTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 7,  status_ok);
		MultiPlexedStatusWord.ProcessorTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 8,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 6 ) {
		MultiPlexedStatusWord.LoopCompletionTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 0,  status_ok);
		MultiPlexedStatusWord.RAMTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 1,  status_ok);
		MultiPlexedStatusWord.CoefficientTableCRCTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
		MultiPlexedStatusWord.ConfigurationTableCRCTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 3,  status_ok);
		MultiPlexedStatusWord.NormalModeSWCRCTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 4,  status_ok);
		MultiPlexedStatusWord.StackOverflowTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 6,  status_ok);
		MultiPlexedStatusWord.WatchdogTimerTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 7,  status_ok);
		MultiPlexedStatusWord.ProcessorTestLatched = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 8,  status_ok);
	}

	if ( MainStatusWord.MuxStatusCounter == 7 ) {
		MultiPlexedStatusWord.SensorTemperature = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord_tmp, 0, 15,  status_ok) * (0.0039));
	}

	if ( MainStatusWord.MuxStatusCounter == 8 ) {
		MultiPlexedStatusWord.MagnetometerTemperature = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord_tmp, 0, 15,  status_ok) * (0.0039));
	}

	if ( MainStatusWord.MuxStatusCounter == 9 ) {
		MultiPlexedStatusWord.DIO1 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 0,  status_ok);
		MultiPlexedStatusWord.DIO2 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 1,  status_ok);
		MultiPlexedStatusWord.DIO3 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 2,  status_ok);
		MultiPlexedStatusWord.DIO4 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord_tmp, 3,  status_ok);
	}
	bb.setOffset(24);	DeltaAngleX = static_cast<double>(bb.get<int32_t>()) * (5.820766E-11);
	bb.setOffset(28);	DeltaAngleY = static_cast<double>(bb.get<int32_t>()) * (5.820766E-11);
	bb.setOffset(32);	DeltaAngleZ = static_cast<double>(bb.get<int32_t>()) * (5.820766E-11);
	bb.setOffset(36);	DeltaVelocityX = static_cast<double>(bb.get<int32_t>()) * (1.135468E-09);
	bb.setOffset(40);	DeltaVelocityY = static_cast<double>(bb.get<int32_t>()) * (1.135468E-09);
	bb.setOffset(44);	DeltaVelocityZ = static_cast<double>(bb.get<int32_t>()) * (1.135468E-09);
	bb.setOffset(48);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 50);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

