#include <include/HGuideAPI.h>
#include <include/Msg_A2.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_A2::SyncByte;
const uint8_t Msg_A2::MessageID;

Msg_A2::Msg_A2()
{
	Default();
}

void Msg_A2::Default()
{
	AngularRateX = 0;
	AngularRateY = 0;
	AngularRateZ = 0;
	LinearAccelerationX = 0;
	LinearAccelerationY = 0;
	LinearAccelerationZ = 0;
	StatusWord1.Default();
	MultiPlexedStatusWord2.Default();
	DeltaAngleX = 0;
	DeltaAngleY = 0;
	DeltaAngleZ = 0;
	DeltaVelocityX = 0;
	DeltaVelocityY = 0;
	DeltaVelocityZ = 0;
	Checksum = 0;
}

bool Msg_A2::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateX / (std::pow(2, -11))));
	bb.setOffset(4);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateY / (std::pow(2, -11))));
	bb.setOffset(6);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateZ / (std::pow(2, -11))));
	bb.setOffset(8);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationX / ((std::pow(2, -5)*0.3048))));
	bb.setOffset(10);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationY / ((std::pow(2, -5)*0.3048))));
	bb.setOffset(12);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationZ / ((std::pow(2, -5)*0.3048))));

	uint16_t StatusWord1_tmp = 0; // temporary variable holding the custom bitfield
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::Pack(StatusWord1_tmp, 0, 3, StatusWord1.StatusWord2ID, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::Pack(StatusWord1_tmp, 4, 7, StatusWord1.Control_Frequency, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::Pack(StatusWord1_tmp, 8, 11, StatusWord1.Guidance_Frequency, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 12, StatusWord1.Gyro_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 13, StatusWord1.Accelerometer_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 14, StatusWord1.Magnetometer_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 15, StatusWord1.CBIT_Status, status_ok);
	bb.setOffset(14);	bb.put(StatusWord1_tmp);


	uint16_t MultiPlexedStatusWord2_tmp = 0; // temporary variable holding the custom bitfield

	if ( StatusWord1.StatusWord2ID == 0 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 0, 7, MultiPlexedStatusWord2.EmbeddedSoftwareVersion, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 8, 11, MultiPlexedStatusWord2.DeviceId, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 12, 15, MultiPlexedStatusWord2.PerformanceGrade, status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 1 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 0, MultiPlexedStatusWord2.Gyro_StatisticsSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 1, MultiPlexedStatusWord2.Gyro_TemperatureSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 2, MultiPlexedStatusWord2.Accel_StatisticsSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 3, MultiPlexedStatusWord2.Accel_TemperatureSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 4, MultiPlexedStatusWord2.Mag_StatisticsSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 5, MultiPlexedStatusWord2.Mag_TemperatureSummary, status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 2 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 0, MultiPlexedStatusWord2.NormalModePrimaryCRC, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 1, MultiPlexedStatusWord2.NormalModeSecondayrCRC, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 2, MultiPlexedStatusWord2.FactoryConfigCRC, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 3, MultiPlexedStatusWord2.FactoryCoefficientCRC, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 4, MultiPlexedStatusWord2.IO_ConfigCRC, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 10, MultiPlexedStatusWord2.PrimaryImageBoot, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 11, MultiPlexedStatusWord2.MemoryTestSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 12, MultiPlexedStatusWord2.ProcessorTestSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 13, MultiPlexedStatusWord2.WdtLoopCompletionSummary, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 14, MultiPlexedStatusWord2.PowerUpBitStatus, status_ok);
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 15, MultiPlexedStatusWord2.ContinuousBitStatus, status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 3 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 0, 15, static_cast<int16_t>CHECK_MAX_INT16(MultiPlexedStatusWord2.DeviceTemperature / (std::pow(2, -8))), status_ok);
	}
	bb.setOffset(16);	bb.put(MultiPlexedStatusWord2_tmp);

	bb.setOffset(18);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleX / (std::pow(2, -33))));
	bb.setOffset(22);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleY / (std::pow(2, -33))));
	bb.setOffset(26);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleZ / (std::pow(2, -33))));
	bb.setOffset(30);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityX / ((std::pow(2, -27)*0.3048))));
	bb.setOffset(34);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityY / ((std::pow(2, -27)*0.3048))));
	bb.setOffset(38);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityZ / ((std::pow(2, -27)*0.3048))));
	int cIndex = 42;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 44);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_A2::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 44) return -2;

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
	bb.setOffset(2);	AngularRateX = static_cast<float>(bb.get<int16_t>()) * (std::pow(2, -11));
	bb.setOffset(4);	AngularRateY = static_cast<float>(bb.get<int16_t>()) * (std::pow(2, -11));
	bb.setOffset(6);	AngularRateZ = static_cast<float>(bb.get<int16_t>()) * (std::pow(2, -11));
	bb.setOffset(8);	LinearAccelerationX = static_cast<float>(bb.get<int16_t>()) * ((std::pow(2, -5)*0.3048));
	bb.setOffset(10);	LinearAccelerationY = static_cast<float>(bb.get<int16_t>()) * ((std::pow(2, -5)*0.3048));
	bb.setOffset(12);	LinearAccelerationZ = static_cast<float>(bb.get<int16_t>()) * ((std::pow(2, -5)*0.3048));

	uint16_t StatusWord1_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(14);	bb.get(StatusWord1_tmp);
	StatusWord1.StatusWord2ID = ECTOS::BIT_UTILITIES::UnPack(StatusWord1_tmp, 0, 3,  status_ok);
	StatusWord1.Control_Frequency = static_cast<control_frequency_t>(ECTOS::BIT_UTILITIES::UnPack(StatusWord1_tmp, 4, 7,  status_ok));
	StatusWord1.Guidance_Frequency = static_cast<guidance_frequency_t>(ECTOS::BIT_UTILITIES::UnPack(StatusWord1_tmp, 8, 11,  status_ok));
	StatusWord1.Gyro_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 12,  status_ok);
	StatusWord1.Accelerometer_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 13,  status_ok);
	StatusWord1.Magnetometer_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 14,  status_ok);
	StatusWord1.CBIT_Status = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 15,  status_ok);

	uint16_t MultiPlexedStatusWord2_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(MultiPlexedStatusWord2_tmp);

	if ( StatusWord1.StatusWord2ID == 0 ) {
		MultiPlexedStatusWord2.EmbeddedSoftwareVersion = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 0, 7,  status_ok);
		MultiPlexedStatusWord2.DeviceId = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 8, 11,  status_ok);
		MultiPlexedStatusWord2.PerformanceGrade = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 12, 15,  status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 1 ) {
		MultiPlexedStatusWord2.Gyro_StatisticsSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 0,  status_ok);
		MultiPlexedStatusWord2.Gyro_TemperatureSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 1,  status_ok);
		MultiPlexedStatusWord2.Accel_StatisticsSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 2,  status_ok);
		MultiPlexedStatusWord2.Accel_TemperatureSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 3,  status_ok);
		MultiPlexedStatusWord2.Mag_StatisticsSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 4,  status_ok);
		MultiPlexedStatusWord2.Mag_TemperatureSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 5,  status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 2 ) {
		MultiPlexedStatusWord2.NormalModePrimaryCRC = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 0,  status_ok);
		MultiPlexedStatusWord2.NormalModeSecondayrCRC = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 1,  status_ok);
		MultiPlexedStatusWord2.FactoryConfigCRC = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 2,  status_ok);
		MultiPlexedStatusWord2.FactoryCoefficientCRC = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 3,  status_ok);
		MultiPlexedStatusWord2.IO_ConfigCRC = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 4,  status_ok);
		MultiPlexedStatusWord2.PrimaryImageBoot = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 10,  status_ok);
		MultiPlexedStatusWord2.MemoryTestSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 11,  status_ok);
		MultiPlexedStatusWord2.ProcessorTestSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 12,  status_ok);
		MultiPlexedStatusWord2.WdtLoopCompletionSummary = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 13,  status_ok);
		MultiPlexedStatusWord2.PowerUpBitStatus = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 14,  status_ok);
		MultiPlexedStatusWord2.ContinuousBitStatus = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 15,  status_ok);
	}

	if ( StatusWord1.StatusWord2ID == 3 ) {
		MultiPlexedStatusWord2.DeviceTemperature = static_cast<float>(ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 0, 15,  status_ok) * (std::pow(2, -8)));
	}
	bb.setOffset(18);	DeltaAngleX = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(22);	DeltaAngleY = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(26);	DeltaAngleZ = static_cast<double>(bb.get<int32_t>()) * (std::pow(2, -33));
	bb.setOffset(30);	DeltaVelocityX = static_cast<double>(bb.get<int32_t>()) * ((std::pow(2, -27)*0.3048));
	bb.setOffset(34);	DeltaVelocityY = static_cast<double>(bb.get<int32_t>()) * ((std::pow(2, -27)*0.3048));
	bb.setOffset(38);	DeltaVelocityZ = static_cast<double>(bb.get<int32_t>()) * ((std::pow(2, -27)*0.3048));
	bb.setOffset(42);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 44);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

