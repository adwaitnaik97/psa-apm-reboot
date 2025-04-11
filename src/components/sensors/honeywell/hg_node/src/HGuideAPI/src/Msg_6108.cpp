#include <include/HGuideAPI.h>
#include <include/Msg_6108.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6108::AddressId;
const uint32_t Msg_6108::MessageId;
const uint32_t Msg_6108::MessageLength;

Msg_6108::Msg_6108()
{
	Default();
}

void Msg_6108::Default()
{
	Checksum = 0;
	gpsTov = 0;
	gps_week = 0;
	InsGnssSummary.Default();
	Latitude = 0;
	Longitude = 0;
	AltitudeHeightAboveEllipsoid = 0;
	NorthVelocity = 0;
	EastVelocity = 0;
	DownVelocity = 0;
	RxClkBias = 0;
	Datum = 0;
	TimeReference = 0;
	NumberOfSvs = 0;
	RTKfixProg = 0;
	CorrInfo = 0;
	SignalInfo = 0;
	PPPInfo = 0;
	LatitudeSTDV = 0;
	LongitudeSTDV = 0;
	AltitudeHeightAboveEllipsoidSTDV = 0;
	NorthVelocitySTDV = 0;
	EastVelocitySTDV = 0;
	DownVelocitySTDV = 0;
	systemTov = 0;
	PDOP = 0;
	HDOP = 0;
	VDOP = 0;
	TDOP = 0;
	Undulation = 0;
}

bool Msg_6108::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 128) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(gpsTov);
	bb.setOffset(24);	bb.put(gps_week);

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
	bb.setOffset(28);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(32);	bb.put(Latitude);
	bb.setOffset(40);	bb.put(Longitude);
	bb.setOffset(48);	bb.put(AltitudeHeightAboveEllipsoid);
	bb.setOffset(56);	bb.put(NorthVelocity);
	bb.setOffset(60);	bb.put(EastVelocity);
	bb.setOffset(64);	bb.put(DownVelocity);
	bb.setOffset(68);	bb.put(RxClkBias);
	bb.setOffset(72);	bb.put(Datum);
	bb.setOffset(73);	bb.put(TimeReference);
	bb.setOffset(74);	bb.put(NumberOfSvs);
	bb.setOffset(75);	bb.put(RTKfixProg);
	bb.setOffset(76);	bb.put(CorrInfo);
	bb.setOffset(80);	bb.put(SignalInfo);
	bb.setOffset(84);	bb.put(PPPInfo);
	bb.setOffset(88);	bb.put(LatitudeSTDV);
	bb.setOffset(92);	bb.put(LongitudeSTDV);
	bb.setOffset(96);	bb.put(AltitudeHeightAboveEllipsoidSTDV);
	bb.setOffset(100);	bb.put(NorthVelocitySTDV);
	bb.setOffset(104);	bb.put(EastVelocitySTDV);
	bb.setOffset(108);	bb.put(DownVelocitySTDV);
	bb.setOffset(112);	bb.put(systemTov);
	bb.setOffset(120);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(PDOP / (0.04)));
	bb.setOffset(121);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(HDOP / (0.04)));
	bb.setOffset(122);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(VDOP / (0.04)));
	bb.setOffset(123);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(TDOP / (0.04)));
	bb.setOffset(124);	bb.put(Undulation);
	Checksum = computeChecksum((uint32_t*)buffer, 32);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6108::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 128) return -2;

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
	bb.setOffset(16);	bb.get(gpsTov);
	bb.setOffset(24);	bb.get(gps_week);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(28);	bb.get(InsGnssSummary_tmp);
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
	bb.setOffset(32);	bb.get(Latitude);
	bb.setOffset(40);	bb.get(Longitude);
	bb.setOffset(48);	bb.get(AltitudeHeightAboveEllipsoid);
	bb.setOffset(56);	bb.get(NorthVelocity);
	bb.setOffset(60);	bb.get(EastVelocity);
	bb.setOffset(64);	bb.get(DownVelocity);
	bb.setOffset(68);	bb.get(RxClkBias);
	bb.setOffset(72);	bb.get(Datum);
	bb.setOffset(73);	bb.get(TimeReference);
	bb.setOffset(74);	bb.get(NumberOfSvs);
	bb.setOffset(75);	bb.get(RTKfixProg);
	bb.setOffset(76);	bb.get(CorrInfo);
	bb.setOffset(80);	bb.get(SignalInfo);
	bb.setOffset(84);	bb.get(PPPInfo);
	bb.setOffset(88);	bb.get(LatitudeSTDV);
	bb.setOffset(92);	bb.get(LongitudeSTDV);
	bb.setOffset(96);	bb.get(AltitudeHeightAboveEllipsoidSTDV);
	bb.setOffset(100);	bb.get(NorthVelocitySTDV);
	bb.setOffset(104);	bb.get(EastVelocitySTDV);
	bb.setOffset(108);	bb.get(DownVelocitySTDV);
	bb.setOffset(112);	bb.get(systemTov);
	bb.setOffset(120);	PDOP = static_cast<float>(bb.get<uint8_t>()) * (0.04);
	bb.setOffset(121);	HDOP = static_cast<float>(bb.get<uint8_t>()) * (0.04);
	bb.setOffset(122);	VDOP = static_cast<float>(bb.get<uint8_t>()) * (0.04);
	bb.setOffset(123);	TDOP = static_cast<float>(bb.get<uint8_t>()) * (0.04);
	bb.setOffset(124);	bb.get(Undulation);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 32);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

