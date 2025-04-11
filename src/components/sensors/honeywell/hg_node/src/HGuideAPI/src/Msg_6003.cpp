#include <include/HGuideAPI.h>
#include <include/Msg_6003.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6003::AddressId;
const uint32_t Msg_6003::MessageId;
const uint32_t Msg_6003::MessageLength;

Msg_6003::Msg_6003()
{
	Default();
}

void Msg_6003::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	InsGnssSummary.Default();
	ImuLeverArmsX = 0;
	ImuLeverArmsY = 0;
	ImuLeverArmsZ = 0;
	VehicleEulerAnglesRoll = 0;
	VehicleEulerAnglesPitch = 0;
	VehicleEulerAnglesTrueHeading = 0;
	CaseToNavQuaterionI = 0;
	CaseToNavQuaterionJ = 0;
	CaseToNavQuaterionK = 0;
	RF1AntennaLeverArmStdv = 0;
	RF1AntennaLeverArmX = 0;
	RF1AntennaLeverArmY = 0;
	RF1AntennaLeverArmZ = 0;
	RF2AuxiliaryAntennaLeverArmX = 0;
	RF2AuxiliaryAntennaLeverArmY = 0;
	RF2AuxiliaryAntennaLeverArmZ = 0;
	RF12LOSPitchEst = 0;
	RF12LOSTrueHeadingEst = 0;
	AntennaBoresightPitchEstStdv = 0;
	AntennaBoresightTrueHeadingEstStdv = 0;
	RF1MainAntennaLeverArmEstX = 0;
	RF1MainAntennaLeverArmEstY = 0;
	RF1MainAntennaLeverArmEstZ = 0;
	RF1MainAntennaLeverArmEstStdvX = 0;
	RF1MainAntennaLeverArmEstStdvY = 0;
	RF1MainAntennaLeverArmEstStdvZ = 0;
	OdometerLeverArmEstX = 0;
	OdometerLeverArmEstY = 0;
	OdometerLeverArmEstZ = 0;
	OdometerLeverArmEstStdvX = 0;
	OdometerLeverArmEstStdvY = 0;
	OdometerLeverArmEstStdvZ = 0;
	OdometerBoresightTrueHeading = 0;
	OdometerBoresightPitch = 0;
	OdometerSFStdv = 0;
	OdometerLeverArmStoredX = 0;
	OdometerLeverArmStoredY = 0;
	OdometerLeverArmStoredZ = 0;
	RF12BoresightTrueHeadingAdjustment = 0;
	RF12BoresightPitchAdjustment = 0;
	CaseToNavQuaterionS = 1.0;
}

bool Msg_6003::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 256) return false;

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

	bb.setOffset(36);	bb.put(ImuLeverArmsX);
	bb.setOffset(40);	bb.put(ImuLeverArmsY);
	bb.setOffset(44);	bb.put(ImuLeverArmsZ);
	bb.setOffset(48);	bb.put(VehicleEulerAnglesRoll);
	bb.setOffset(52);	bb.put(VehicleEulerAnglesPitch);
	bb.setOffset(56);	bb.put(VehicleEulerAnglesTrueHeading);
	bb.setOffset(60);	bb.put(CaseToNavQuaterionS);
	bb.setOffset(64);	bb.put(CaseToNavQuaterionI);
	bb.setOffset(68);	bb.put(CaseToNavQuaterionJ);
	bb.setOffset(72);	bb.put(CaseToNavQuaterionK);
	bb.setOffset(76);	bb.put(RF1AntennaLeverArmStdv);
	bb.setOffset(80);	bb.put(RF1AntennaLeverArmX);
	bb.setOffset(84);	bb.put(RF1AntennaLeverArmY);
	bb.setOffset(88);	bb.put(RF1AntennaLeverArmZ);
	bb.setOffset(92);	bb.put(RF2AuxiliaryAntennaLeverArmX);
	bb.setOffset(96);	bb.put(RF2AuxiliaryAntennaLeverArmY);
	bb.setOffset(100);	bb.put(RF2AuxiliaryAntennaLeverArmZ);
	bb.setOffset(104);	bb.put(RF12LOSPitchEst);
	bb.setOffset(108);	bb.put(RF12LOSTrueHeadingEst);
	bb.setOffset(112);	bb.put(AntennaBoresightPitchEstStdv);
	bb.setOffset(116);	bb.put(AntennaBoresightTrueHeadingEstStdv);
	bb.setOffset(120);	bb.put(RF1MainAntennaLeverArmEstX);
	bb.setOffset(124);	bb.put(RF1MainAntennaLeverArmEstY);
	bb.setOffset(128);	bb.put(RF1MainAntennaLeverArmEstZ);
	bb.setOffset(132);	bb.put(RF1MainAntennaLeverArmEstStdvX);
	bb.setOffset(136);	bb.put(RF1MainAntennaLeverArmEstStdvY);
	bb.setOffset(140);	bb.put(RF1MainAntennaLeverArmEstStdvZ);
	bb.setOffset(144);	bb.put(OdometerLeverArmEstX);
	bb.setOffset(148);	bb.put(OdometerLeverArmEstY);
	bb.setOffset(152);	bb.put(OdometerLeverArmEstZ);
	bb.setOffset(156);	bb.put(OdometerLeverArmEstStdvX);
	bb.setOffset(160);	bb.put(OdometerLeverArmEstStdvY);
	bb.setOffset(164);	bb.put(OdometerLeverArmEstStdvZ);
	bb.setOffset(168);	bb.put(OdometerBoresightTrueHeading);
	bb.setOffset(172);	bb.put(OdometerBoresightPitch);
	bb.setOffset(176);	bb.put(OdometerSFStdv);
	bb.setOffset(180);	bb.put(OdometerLeverArmStoredX);
	bb.setOffset(184);	bb.put(OdometerLeverArmStoredY);
	bb.setOffset(188);	bb.put(OdometerLeverArmStoredZ);
	bb.setOffset(192);	bb.put(RF12BoresightTrueHeadingAdjustment);
	bb.setOffset(196);	bb.put(RF12BoresightPitchAdjustment);
	Checksum = computeChecksum((uint32_t*)buffer, 64);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6003::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 256) return -2;

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
	bb.setOffset(36);	bb.get(ImuLeverArmsX);
	bb.setOffset(40);	bb.get(ImuLeverArmsY);
	bb.setOffset(44);	bb.get(ImuLeverArmsZ);
	bb.setOffset(48);	bb.get(VehicleEulerAnglesRoll);
	bb.setOffset(52);	bb.get(VehicleEulerAnglesPitch);
	bb.setOffset(56);	bb.get(VehicleEulerAnglesTrueHeading);
	bb.setOffset(60);	bb.get(CaseToNavQuaterionS);
	bb.setOffset(64);	bb.get(CaseToNavQuaterionI);
	bb.setOffset(68);	bb.get(CaseToNavQuaterionJ);
	bb.setOffset(72);	bb.get(CaseToNavQuaterionK);
	bb.setOffset(76);	bb.get(RF1AntennaLeverArmStdv);
	bb.setOffset(80);	bb.get(RF1AntennaLeverArmX);
	bb.setOffset(84);	bb.get(RF1AntennaLeverArmY);
	bb.setOffset(88);	bb.get(RF1AntennaLeverArmZ);
	bb.setOffset(92);	bb.get(RF2AuxiliaryAntennaLeverArmX);
	bb.setOffset(96);	bb.get(RF2AuxiliaryAntennaLeverArmY);
	bb.setOffset(100);	bb.get(RF2AuxiliaryAntennaLeverArmZ);
	bb.setOffset(104);	bb.get(RF12LOSPitchEst);
	bb.setOffset(108);	bb.get(RF12LOSTrueHeadingEst);
	bb.setOffset(112);	bb.get(AntennaBoresightPitchEstStdv);
	bb.setOffset(116);	bb.get(AntennaBoresightTrueHeadingEstStdv);
	bb.setOffset(120);	bb.get(RF1MainAntennaLeverArmEstX);
	bb.setOffset(124);	bb.get(RF1MainAntennaLeverArmEstY);
	bb.setOffset(128);	bb.get(RF1MainAntennaLeverArmEstZ);
	bb.setOffset(132);	bb.get(RF1MainAntennaLeverArmEstStdvX);
	bb.setOffset(136);	bb.get(RF1MainAntennaLeverArmEstStdvY);
	bb.setOffset(140);	bb.get(RF1MainAntennaLeverArmEstStdvZ);
	bb.setOffset(144);	bb.get(OdometerLeverArmEstX);
	bb.setOffset(148);	bb.get(OdometerLeverArmEstY);
	bb.setOffset(152);	bb.get(OdometerLeverArmEstZ);
	bb.setOffset(156);	bb.get(OdometerLeverArmEstStdvX);
	bb.setOffset(160);	bb.get(OdometerLeverArmEstStdvY);
	bb.setOffset(164);	bb.get(OdometerLeverArmEstStdvZ);
	bb.setOffset(168);	bb.get(OdometerBoresightTrueHeading);
	bb.setOffset(172);	bb.get(OdometerBoresightPitch);
	bb.setOffset(176);	bb.get(OdometerSFStdv);
	bb.setOffset(180);	bb.get(OdometerLeverArmStoredX);
	bb.setOffset(184);	bb.get(OdometerLeverArmStoredY);
	bb.setOffset(188);	bb.get(OdometerLeverArmStoredZ);
	bb.setOffset(192);	bb.get(RF12BoresightTrueHeadingAdjustment);
	bb.setOffset(196);	bb.get(RF12BoresightPitchAdjustment);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 64);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

