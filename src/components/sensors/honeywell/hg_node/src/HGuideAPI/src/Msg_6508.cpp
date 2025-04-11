#include <include/HGuideAPI.h>
#include <include/Msg_6508.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6508::AddressId;
const uint32_t Msg_6508::MessageId;
const uint32_t Msg_6508::MessageLength;

Msg_6508::Msg_6508()
{
	Default();
}

void Msg_6508::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	InsGnssSummary.Default();
	RF1AntSatSearch.Default();
	RF1AntSatLock.Default();
	RF2AntSatSearch.Default();
	RF2AntSatLock.Default();
}

bool Msg_6508::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 104) return false;

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
	bb.setOffset(32);	bb.put(gps_week);

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
	bb.setOffset(36);	bb.put(InsGnssSummary_tmp);


	uint32_t RF1AntSatSearch_tmp[4] = { 0 }; // buffer holding the custom bitfield
	RF1AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[0], 0, 15, RF1AntSatSearch.TotalSats, status_ok);
	RF1AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[0], 16, 23, RF1AntSatSearch.NumGps, status_ok);
	RF1AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[0], 24, 31, RF1AntSatSearch.NumGlo, status_ok);
	RF1AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[1], 0, 7, RF1AntSatSearch.NumGal, status_ok);
	RF1AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[1], 8, 15, RF1AntSatSearch.NumBds, status_ok);
	RF1AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[1], 16, 23, RF1AntSatSearch.NumSbas, status_ok);
	RF1AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[1], 24, 31, RF1AntSatSearch.NumQzss, status_ok);
	RF1AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[2], 0, 7, RF1AntSatSearch.NumIrnss, status_ok);
	RF1AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[2], 8, 15, RF1AntSatSearch.NumLband, status_ok);
	RF1AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatSearch_tmp[2], 16, 23, RF1AntSatSearch.NumUnknown, status_ok);
	bb.setOffset(40);	bb.put(RF1AntSatSearch_tmp);


	uint32_t RF1AntSatLock_tmp[4] = { 0 }; // buffer holding the custom bitfield
	RF1AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[0], 0, 15, RF1AntSatLock.TotalSats, status_ok);
	RF1AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[0], 16, 23, RF1AntSatLock.NumGps, status_ok);
	RF1AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[0], 24, 31, RF1AntSatLock.NumGlo, status_ok);
	RF1AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[1], 0, 7, RF1AntSatLock.NumGal, status_ok);
	RF1AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[1], 8, 15, RF1AntSatLock.NumBds, status_ok);
	RF1AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[1], 16, 23, RF1AntSatLock.NumSbas, status_ok);
	RF1AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[1], 24, 31, RF1AntSatLock.NumQzss, status_ok);
	RF1AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[2], 0, 7, RF1AntSatLock.NumIrnss, status_ok);
	RF1AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[2], 8, 15, RF1AntSatLock.NumLband, status_ok);
	RF1AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF1AntSatLock_tmp[2], 16, 23, RF1AntSatLock.NumUnknown, status_ok);
	bb.setOffset(56);	bb.put(RF1AntSatLock_tmp);


	uint32_t RF2AntSatSearch_tmp[4] = { 0 }; // buffer holding the custom bitfield
	RF2AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[0], 0, 15, RF2AntSatSearch.TotalSats, status_ok);
	RF2AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[0], 16, 23, RF2AntSatSearch.NumGps, status_ok);
	RF2AntSatSearch_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[0], 24, 31, RF2AntSatSearch.NumGlo, status_ok);
	RF2AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[1], 0, 7, RF2AntSatSearch.NumGal, status_ok);
	RF2AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[1], 8, 15, RF2AntSatSearch.NumBds, status_ok);
	RF2AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[1], 16, 23, RF2AntSatSearch.NumSbas, status_ok);
	RF2AntSatSearch_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[1], 24, 31, RF2AntSatSearch.NumQzss, status_ok);
	RF2AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[2], 0, 7, RF2AntSatSearch.NumIrnss, status_ok);
	RF2AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[2], 8, 15, RF2AntSatSearch.NumLband, status_ok);
	RF2AntSatSearch_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatSearch_tmp[2], 16, 23, RF2AntSatSearch.NumUnknown, status_ok);
	bb.setOffset(72);	bb.put(RF2AntSatSearch_tmp);


	uint32_t RF2AntSatLock_tmp[4] = { 0 }; // buffer holding the custom bitfield
	RF2AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[0], 0, 15, RF2AntSatLock.TotalSats, status_ok);
	RF2AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[0], 16, 23, RF2AntSatLock.NumGps, status_ok);
	RF2AntSatLock_tmp[0] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[0], 24, 31, RF2AntSatLock.NumGlo, status_ok);
	RF2AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[1], 0, 7, RF2AntSatLock.NumGal, status_ok);
	RF2AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[1], 8, 15, RF2AntSatLock.NumBds, status_ok);
	RF2AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[1], 16, 23, RF2AntSatLock.NumSbas, status_ok);
	RF2AntSatLock_tmp[1] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[1], 24, 31, RF2AntSatLock.NumQzss, status_ok);
	RF2AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[2], 0, 7, RF2AntSatLock.NumIrnss, status_ok);
	RF2AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[2], 8, 15, RF2AntSatLock.NumLband, status_ok);
	RF2AntSatLock_tmp[2] = ECTOS::BIT_UTILITIES::Pack(RF2AntSatLock_tmp[2], 16, 23, RF2AntSatLock.NumUnknown, status_ok);
	bb.setOffset(88);	bb.put(RF2AntSatLock_tmp);

	Checksum = computeChecksum((uint32_t*)buffer, 26);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6508::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 104) return -2;

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
	bb.setOffset(32);	bb.get(gps_week);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(36);	bb.get(InsGnssSummary_tmp);
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

	uint32_t RF1AntSatSearch_tmp[4] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(40);	bb.get(RF1AntSatSearch_tmp);
	RF1AntSatSearch.TotalSats = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[0], 0, 15,  status_ok);
	RF1AntSatSearch.NumGps = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[0], 16, 23,  status_ok);
	RF1AntSatSearch.NumGlo = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[0], 24, 31,  status_ok);
	RF1AntSatSearch.NumGal = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[1], 0, 7,  status_ok);
	RF1AntSatSearch.NumBds = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[1], 8, 15,  status_ok);
	RF1AntSatSearch.NumSbas = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[1], 16, 23,  status_ok);
	RF1AntSatSearch.NumQzss = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[1], 24, 31,  status_ok);
	RF1AntSatSearch.NumIrnss = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[2], 0, 7,  status_ok);
	RF1AntSatSearch.NumLband = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[2], 8, 15,  status_ok);
	RF1AntSatSearch.NumUnknown = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatSearch_tmp[2], 16, 23,  status_ok);

	uint32_t RF1AntSatLock_tmp[4] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(56);	bb.get(RF1AntSatLock_tmp);
	RF1AntSatLock.TotalSats = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[0], 0, 15,  status_ok);
	RF1AntSatLock.NumGps = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[0], 16, 23,  status_ok);
	RF1AntSatLock.NumGlo = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[0], 24, 31,  status_ok);
	RF1AntSatLock.NumGal = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[1], 0, 7,  status_ok);
	RF1AntSatLock.NumBds = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[1], 8, 15,  status_ok);
	RF1AntSatLock.NumSbas = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[1], 16, 23,  status_ok);
	RF1AntSatLock.NumQzss = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[1], 24, 31,  status_ok);
	RF1AntSatLock.NumIrnss = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[2], 0, 7,  status_ok);
	RF1AntSatLock.NumLband = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[2], 8, 15,  status_ok);
	RF1AntSatLock.NumUnknown = ECTOS::BIT_UTILITIES::UnPack(RF1AntSatLock_tmp[2], 16, 23,  status_ok);

	uint32_t RF2AntSatSearch_tmp[4] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(72);	bb.get(RF2AntSatSearch_tmp);
	RF2AntSatSearch.TotalSats = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[0], 0, 15,  status_ok);
	RF2AntSatSearch.NumGps = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[0], 16, 23,  status_ok);
	RF2AntSatSearch.NumGlo = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[0], 24, 31,  status_ok);
	RF2AntSatSearch.NumGal = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[1], 0, 7,  status_ok);
	RF2AntSatSearch.NumBds = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[1], 8, 15,  status_ok);
	RF2AntSatSearch.NumSbas = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[1], 16, 23,  status_ok);
	RF2AntSatSearch.NumQzss = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[1], 24, 31,  status_ok);
	RF2AntSatSearch.NumIrnss = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[2], 0, 7,  status_ok);
	RF2AntSatSearch.NumLband = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[2], 8, 15,  status_ok);
	RF2AntSatSearch.NumUnknown = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatSearch_tmp[2], 16, 23,  status_ok);

	uint32_t RF2AntSatLock_tmp[4] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(88);	bb.get(RF2AntSatLock_tmp);
	RF2AntSatLock.TotalSats = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[0], 0, 15,  status_ok);
	RF2AntSatLock.NumGps = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[0], 16, 23,  status_ok);
	RF2AntSatLock.NumGlo = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[0], 24, 31,  status_ok);
	RF2AntSatLock.NumGal = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[1], 0, 7,  status_ok);
	RF2AntSatLock.NumBds = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[1], 8, 15,  status_ok);
	RF2AntSatLock.NumSbas = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[1], 16, 23,  status_ok);
	RF2AntSatLock.NumQzss = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[1], 24, 31,  status_ok);
	RF2AntSatLock.NumIrnss = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[2], 0, 7,  status_ok);
	RF2AntSatLock.NumLband = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[2], 8, 15,  status_ok);
	RF2AntSatLock.NumUnknown = ECTOS::BIT_UTILITIES::UnPack(RF2AntSatLock_tmp[2], 16, 23,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 26);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

