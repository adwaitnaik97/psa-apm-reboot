#include <include/HGuideAPI.h>
#include <include/Msg_6111.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6111::AddressId;
const uint32_t Msg_6111::MessageId;
const uint32_t Msg_6111::MessageLength;

Msg_6111::Msg_6111()
{
	Default();
}

void Msg_6111::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	InsGnssSummary.Default();
	Latitude = 0;
	Longitude = 0;
	Test1_RotationX = 0;
	Test1_RotationY = 0;
	Test1_RotationZ = 0;
	Test1_RotationNormRate = 0;
	Test1_RotationNormRateThreshold = 0;
	Test2_SpeedValid = 0;
	Test2_Speed = 0;
	Test2_SpeedThreshold = 0;
	Test3_AngularRateInstantX = 0;
	Test3_AngularRateInstantY = 0;
	Test3_AngularRateInstantZ = 0;
	Test3_InstantFilterBandwidth = 0;
	Test3_AngularRateNominalX = 0;
	Test3_AngularRateNominalY = 0;
	Test3_AngularRateNominalZ = 0;
	Test3_NominalFilterBandwidth = 0;
	Test3_AngularRateX = 0;
	Test3_AngularRateY = 0;
	Test3_AngularRateZ = 0;
	Test3_AngularRateThreshold = 0;
	Test4_LinearAcceleration = 0;
	Test4_LinearAccelerationThreshold = 0;
	Test5_OdometerDeltaDistance = 0;
	Test5_OdometerDeltaDistanceThreshold = 0;
	SettlingTime_Odometer = 0;
	SettlingTime_Stationary = 0;
	SettlingTime_Threshold = 0;
}

bool Msg_6111::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.put(systemTov);
	bb.setOffset(24);	bb.put(gpsTov);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 0, 3, InsGnssSummary.INSMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 4, InsGnssSummary.INSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 5, InsGnssSummary.IMUStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 6, InsGnssSummary.GNSSStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 7, InsGnssSummary.MotionDetectActive, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 8, InsGnssSummary.StationaryMeasurementsOn, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 9, InsGnssSummary.MDT1RotationRate, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 10, InsGnssSummary.MDT2SpeedSTDV, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 11, InsGnssSummary.MDT3AngularRateInstantBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 12, InsGnssSummary.MDT4LinearAccelerationBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 13, InsGnssSummary.MDT5OdometerBit, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::PackBool(InsGnssSummary_tmp, 16, InsGnssSummary.MDNavigationMode, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 17, 19, InsGnssSummary.NavSmoothingStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 28, 31, InsGnssSummary.GPSMode, status_ok);
	bb.setOffset(32);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(36);	bb.put(Latitude);
	bb.setOffset(44);	bb.put(Longitude);
	bb.setOffset(52);	bb.put(Test1_RotationX);
	bb.setOffset(56);	bb.put(Test1_RotationY);
	bb.setOffset(60);	bb.put(Test1_RotationZ);
	bb.setOffset(64);	bb.put(Test1_RotationNormRate);
	bb.setOffset(68);	bb.put(Test1_RotationNormRateThreshold);
	bb.setOffset(72);	bb.put(Test2_SpeedValid);
	bb.setOffset(76);	bb.put(Test2_Speed);
	bb.setOffset(80);	bb.put(Test2_SpeedThreshold);
	bb.setOffset(84);	bb.put(Test3_AngularRateInstantX);
	bb.setOffset(88);	bb.put(Test3_AngularRateInstantY);
	bb.setOffset(92);	bb.put(Test3_AngularRateInstantZ);
	bb.setOffset(96);	bb.put(Test3_InstantFilterBandwidth);
	bb.setOffset(100);	bb.put(Test3_AngularRateNominalX);
	bb.setOffset(104);	bb.put(Test3_AngularRateNominalY);
	bb.setOffset(108);	bb.put(Test3_AngularRateNominalZ);
	bb.setOffset(112);	bb.put(Test3_NominalFilterBandwidth);
	bb.setOffset(116);	bb.put(Test3_AngularRateX);
	bb.setOffset(120);	bb.put(Test3_AngularRateY);
	bb.setOffset(124);	bb.put(Test3_AngularRateZ);
	bb.setOffset(128);	bb.put(Test3_AngularRateThreshold);
	bb.setOffset(132);	bb.put(Test4_LinearAcceleration);
	bb.setOffset(136);	bb.put(Test4_LinearAccelerationThreshold);
	bb.setOffset(140);	bb.put(Test5_OdometerDeltaDistance);
	bb.setOffset(144);	bb.put(Test5_OdometerDeltaDistanceThreshold);
	bb.setOffset(148);	bb.put(SettlingTime_Odometer);
	bb.setOffset(152);	bb.put(SettlingTime_Stationary);
	bb.setOffset(156);	bb.put(SettlingTime_Threshold);
	Checksum = computeChecksum((uint32_t*)buffer, 40);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6111::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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
	bb.setOffset(16);	bb.get(systemTov);
	bb.setOffset(24);	bb.get(gpsTov);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(32);	bb.get(InsGnssSummary_tmp);
	InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 0, 3,  status_ok));
	InsGnssSummary.INSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 4,  status_ok);
	InsGnssSummary.IMUStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 5,  status_ok);
	InsGnssSummary.GNSSStatus = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 6,  status_ok);
	InsGnssSummary.MotionDetectActive = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 7,  status_ok);
	InsGnssSummary.StationaryMeasurementsOn = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 8,  status_ok);
	InsGnssSummary.MDT1RotationRate = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 9,  status_ok);
	InsGnssSummary.MDT2SpeedSTDV = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 10,  status_ok);
	InsGnssSummary.MDT3AngularRateInstantBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 11,  status_ok);
	InsGnssSummary.MDT4LinearAccelerationBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 12,  status_ok);
	InsGnssSummary.MDT5OdometerBit = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 13,  status_ok);
	InsGnssSummary.MDNavigationMode = ECTOS::BIT_UTILITIES::UnPackBool(InsGnssSummary_tmp, 16,  status_ok);
	InsGnssSummary.NavSmoothingStatus = ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 17, 19,  status_ok);
	InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 28, 31,  status_ok));
	bb.setOffset(36);	bb.get(Latitude);
	bb.setOffset(44);	bb.get(Longitude);
	bb.setOffset(52);	bb.get(Test1_RotationX);
	bb.setOffset(56);	bb.get(Test1_RotationY);
	bb.setOffset(60);	bb.get(Test1_RotationZ);
	bb.setOffset(64);	bb.get(Test1_RotationNormRate);
	bb.setOffset(68);	bb.get(Test1_RotationNormRateThreshold);
	bb.setOffset(72);	bb.get(Test2_SpeedValid);
	bb.setOffset(76);	bb.get(Test2_Speed);
	bb.setOffset(80);	bb.get(Test2_SpeedThreshold);
	bb.setOffset(84);	bb.get(Test3_AngularRateInstantX);
	bb.setOffset(88);	bb.get(Test3_AngularRateInstantY);
	bb.setOffset(92);	bb.get(Test3_AngularRateInstantZ);
	bb.setOffset(96);	bb.get(Test3_InstantFilterBandwidth);
	bb.setOffset(100);	bb.get(Test3_AngularRateNominalX);
	bb.setOffset(104);	bb.get(Test3_AngularRateNominalY);
	bb.setOffset(108);	bb.get(Test3_AngularRateNominalZ);
	bb.setOffset(112);	bb.get(Test3_NominalFilterBandwidth);
	bb.setOffset(116);	bb.get(Test3_AngularRateX);
	bb.setOffset(120);	bb.get(Test3_AngularRateY);
	bb.setOffset(124);	bb.get(Test3_AngularRateZ);
	bb.setOffset(128);	bb.get(Test3_AngularRateThreshold);
	bb.setOffset(132);	bb.get(Test4_LinearAcceleration);
	bb.setOffset(136);	bb.get(Test4_LinearAccelerationThreshold);
	bb.setOffset(140);	bb.get(Test5_OdometerDeltaDistance);
	bb.setOffset(144);	bb.get(Test5_OdometerDeltaDistanceThreshold);
	bb.setOffset(148);	bb.get(SettlingTime_Odometer);
	bb.setOffset(152);	bb.get(SettlingTime_Stationary);
	bb.setOffset(156);	bb.get(SettlingTime_Threshold);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 40);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

