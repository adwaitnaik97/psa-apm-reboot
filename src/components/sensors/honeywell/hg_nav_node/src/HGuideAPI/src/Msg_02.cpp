#include <include/HGuideAPI.h>
#include <include/Msg_02.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint8_t Msg_02::SyncByte;
const uint8_t Msg_02::MessageID;

Msg_02::Msg_02()
{
	Default();
}

void Msg_02::Default()
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

bool Msg_02::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 44) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(SyncByte);
	bb.setOffset(1);	bb.put(MessageID);
	bb.setOffset(2);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateX / (0.00057220459)));
	bb.setOffset(4);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateY / (0.00057220459)));
	bb.setOffset(6);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(AngularRateZ / (0.00057220459)));
	bb.setOffset(8);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationX / (0.011162109375)));
	bb.setOffset(10);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationY / (0.011162109375)));
	bb.setOffset(12);	bb.put(static_cast<int16_t>CHECK_MAX_INT16(LinearAccelerationZ / (0.011162109375)));

	uint16_t StatusWord1_tmp = 0; // temporary variable holding the custom bitfield
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::Pack(StatusWord1_tmp, 0, 1, StatusWord1.Counter, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::Pack(StatusWord1_tmp, 2, 3, StatusWord1.BITmodeIndicator, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 4, StatusWord1.IMU_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 5, StatusWord1.Gyro_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 6, StatusWord1.Accel_BIT_Summary, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 7, StatusWord1.GyroVoltage_BIT, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 8, StatusWord1.GyroX_BIT, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 9, StatusWord1.GyroY_BIT, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 10, StatusWord1.GyroZ_BIT, status_ok);
	StatusWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(StatusWord1_tmp, 15, StatusWord1.IMU_OK, status_ok);
	bb.setOffset(14);	bb.put(StatusWord1_tmp);


	uint16_t MultiPlexedStatusWord2_tmp = 0; // temporary variable holding the custom bitfield
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 8, MultiPlexedStatusWord2.GyroHealth1, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 9, MultiPlexedStatusWord2.StartDataFlag, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 10, MultiPlexedStatusWord2.ProcessTest, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 11, MultiPlexedStatusWord2.MemoryTest, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 12, MultiPlexedStatusWord2.ElectronicsTest, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 13, MultiPlexedStatusWord2.GyroHealth2, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 14, MultiPlexedStatusWord2.AcceHealth, status_ok);
	MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MultiPlexedStatusWord2_tmp, 15, MultiPlexedStatusWord2.StatusWord2ID, status_ok);

	if ( MultiPlexedStatusWord2.StatusWord2ID == 0 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 0, 7, MultiPlexedStatusWord2.EmbeddedSoftwareVersion, status_ok);
	}

	if ( MultiPlexedStatusWord2.StatusWord2ID == 1 ) {
		MultiPlexedStatusWord2_tmp = ECTOS::BIT_UTILITIES::Pack(MultiPlexedStatusWord2_tmp, 0, 7, MultiPlexedStatusWord2.AccelXTemperature, status_ok);
	}
	bb.setOffset(16);	bb.put(MultiPlexedStatusWord2_tmp);

	bb.setOffset(18);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleX / (1.164153E-10)));
	bb.setOffset(22);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleY / (1.164153E-10)));
	bb.setOffset(26);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaAngleZ / (1.164153E-10)));
	bb.setOffset(30);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityX / (2.270937E-09)));
	bb.setOffset(34);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityY / (2.270937E-09)));
	bb.setOffset(38);	bb.put(static_cast<int32_t>CHECK_MAX_INT32(DeltaVelocityZ / (2.270937E-09)));
	int cIndex = 42;
	Checksum = 0;
	Checksum = computeChecksum16Bit((uint8_t*)buffer, 44);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_02::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(2);	AngularRateX = static_cast<float>(bb.get<int16_t>()) * (0.00057220459);
	bb.setOffset(4);	AngularRateY = static_cast<float>(bb.get<int16_t>()) * (0.00057220459);
	bb.setOffset(6);	AngularRateZ = static_cast<float>(bb.get<int16_t>()) * (0.00057220459);
	bb.setOffset(8);	LinearAccelerationX = static_cast<float>(bb.get<int16_t>()) * (0.011162109375);
	bb.setOffset(10);	LinearAccelerationY = static_cast<float>(bb.get<int16_t>()) * (0.011162109375);
	bb.setOffset(12);	LinearAccelerationZ = static_cast<float>(bb.get<int16_t>()) * (0.011162109375);

	uint16_t StatusWord1_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(14);	bb.get(StatusWord1_tmp);
	StatusWord1.Counter = ECTOS::BIT_UTILITIES::UnPack(StatusWord1_tmp, 0, 1,  status_ok);
	StatusWord1.BITmodeIndicator = ECTOS::BIT_UTILITIES::UnPack(StatusWord1_tmp, 2, 3,  status_ok);
	StatusWord1.IMU_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 4,  status_ok);
	StatusWord1.Gyro_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 5,  status_ok);
	StatusWord1.Accel_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 6,  status_ok);
	StatusWord1.GyroVoltage_BIT = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 7,  status_ok);
	StatusWord1.GyroX_BIT = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 8,  status_ok);
	StatusWord1.GyroY_BIT = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 9,  status_ok);
	StatusWord1.GyroZ_BIT = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 10,  status_ok);
	StatusWord1.IMU_OK = ECTOS::BIT_UTILITIES::UnPackBool(StatusWord1_tmp, 15,  status_ok);

	uint16_t MultiPlexedStatusWord2_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(MultiPlexedStatusWord2_tmp);
	MultiPlexedStatusWord2.GyroHealth1 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 8,  status_ok);
	MultiPlexedStatusWord2.StartDataFlag = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 9,  status_ok);
	MultiPlexedStatusWord2.ProcessTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 10,  status_ok);
	MultiPlexedStatusWord2.MemoryTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 11,  status_ok);
	MultiPlexedStatusWord2.ElectronicsTest = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 12,  status_ok);
	MultiPlexedStatusWord2.GyroHealth2 = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 13,  status_ok);
	MultiPlexedStatusWord2.AcceHealth = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 14,  status_ok);
	MultiPlexedStatusWord2.StatusWord2ID = ECTOS::BIT_UTILITIES::UnPackBool(MultiPlexedStatusWord2_tmp, 15,  status_ok);

	if ( MultiPlexedStatusWord2.StatusWord2ID == 0 ) {
		MultiPlexedStatusWord2.EmbeddedSoftwareVersion = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 0, 7,  status_ok);
	}

	if ( MultiPlexedStatusWord2.StatusWord2ID == 1 ) {
		MultiPlexedStatusWord2.AccelXTemperature = ECTOS::BIT_UTILITIES::UnPack(MultiPlexedStatusWord2_tmp, 0, 7,  status_ok);
	}
	bb.setOffset(18);	DeltaAngleX = static_cast<double>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(22);	DeltaAngleY = static_cast<double>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(26);	DeltaAngleZ = static_cast<double>(bb.get<int32_t>()) * (1.164153E-10);
	bb.setOffset(30);	DeltaVelocityX = static_cast<double>(bb.get<int32_t>()) * (2.270937E-09);
	bb.setOffset(34);	DeltaVelocityY = static_cast<double>(bb.get<int32_t>()) * (2.270937E-09);
	bb.setOffset(38);	DeltaVelocityZ = static_cast<double>(bb.get<int32_t>()) * (2.270937E-09);
	bb.setOffset(42);	bb.get(Checksum);
	uint16_t computedCRC = computeChecksum16Bit((uint8_t*)buffer, 44);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

