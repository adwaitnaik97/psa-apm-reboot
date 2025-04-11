#include <include/HGuideAPI.h>
#include <include/Msg_6505.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_6505::AddressId;
const uint32_t Msg_6505::MessageId;
const uint32_t Msg_6505::MessageLength;

Msg_6505::Msg_6505()
{
	Default();
}

void Msg_6505::Default()
{
	Checksum = 0;
	Constellation = static_cast<constellation_enum_t>(UNKNOWN_CONSTELLATION);
	Used_in_soln = 0;
	Final_msg_in_set = 0;
	prn_slot = 0;
	elevation = 0;
	azimuth = 0;
	freq_band_1 = 0;
	channel_1 = 0;
	carrier_noise_1 = 0;
	freq_band_2 = 0;
	channel_2 = 0;
	carrier_noise_2 = 0;
	freq_band_3 = 0;
	channel_3 = 0;
	carrier_noise_3 = 0;
	freq_band_4 = 0;
	channel_4 = 0;
	carrier_noise_4 = 0;
	freq_band_5 = 0;
	channel_5 = 0;
	carrier_noise_5 = 0;
	freq_band_6 = 0;
	channel_6 = 0;
	carrier_noise_6 = 0;
	InsGnssSummary.Default();
	gps_week = 0;
	gpsTov = 0;
}

bool Msg_6505::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 64) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;

	uint32_t bitfieldAtByte16 = 0; // temporary variable holding the bitfield
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::Pack(bitfieldAtByte16, 0, 2, static_cast<uint32_t>(Constellation), status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 3, Used_in_soln, status_ok);
	bitfieldAtByte16 = ECTOS::BIT_UTILITIES::PackBool(bitfieldAtByte16, 4, Final_msg_in_set, status_ok);
	bb.setOffset(16);	bb.put(bitfieldAtByte16);

	bb.setOffset(20);	bb.put(prn_slot);
	bb.setOffset(21);	bb.put(static_cast<uint8_t>CHECK_MAX_UINT8(elevation / (0.3529)));
	bb.setOffset(22);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(azimuth / (0.00549)));
	bb.setOffset(24);	bb.put(freq_band_1);
	bb.setOffset(25);	bb.put(channel_1);
	bb.setOffset(26);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_1 / (0.003)));
	bb.setOffset(28);	bb.put(freq_band_2);
	bb.setOffset(29);	bb.put(channel_2);
	bb.setOffset(30);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_2 / (0.003)));
	bb.setOffset(32);	bb.put(freq_band_3);
	bb.setOffset(33);	bb.put(channel_3);
	bb.setOffset(34);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_3 / (0.003)));
	bb.setOffset(36);	bb.put(freq_band_4);
	bb.setOffset(37);	bb.put(channel_4);
	bb.setOffset(38);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_4 / (0.003)));
	bb.setOffset(40);	bb.put(freq_band_5);
	bb.setOffset(41);	bb.put(channel_5);
	bb.setOffset(42);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_5 / (0.003)));
	bb.setOffset(44);	bb.put(freq_band_6);
	bb.setOffset(45);	bb.put(channel_6);
	bb.setOffset(46);	bb.put(static_cast<uint16_t>CHECK_MAX_UINT16(carrier_noise_6 / (0.003)));

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
	bb.setOffset(48);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(52);	bb.put(gps_week);
	bb.setOffset(56);	bb.put(gpsTov);
	Checksum = computeChecksum((uint32_t*)buffer, 16);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_6505::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 64) return -2;

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
	Constellation = static_cast<constellation_enum_t>(ECTOS::BIT_UTILITIES::UnPack(bitfieldAtByte16, 0, 2,  status_ok));
	Used_in_soln = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 3,  status_ok);
	Final_msg_in_set = ECTOS::BIT_UTILITIES::UnPackBool(bitfieldAtByte16, 4,  status_ok);
	bb.setOffset(20);	bb.get(prn_slot);
	bb.setOffset(21);	elevation = static_cast<float>(bb.get<uint8_t>()) * (0.3529);
	bb.setOffset(22);	azimuth = static_cast<float>(bb.get<uint16_t>()) * (0.00549);
	bb.setOffset(24);	bb.get(freq_band_1);
	bb.setOffset(25);	bb.get(channel_1);
	bb.setOffset(26);	carrier_noise_1 = static_cast<float>(bb.get<uint16_t>()) * (0.003);
	bb.setOffset(28);	bb.get(freq_band_2);
	bb.setOffset(29);	bb.get(channel_2);
	bb.setOffset(30);	carrier_noise_2 = static_cast<float>(bb.get<uint16_t>()) * (0.003);
	bb.setOffset(32);	bb.get(freq_band_3);
	bb.setOffset(33);	bb.get(channel_3);
	bb.setOffset(34);	carrier_noise_3 = static_cast<float>(bb.get<uint16_t>()) * (0.003);
	bb.setOffset(36);	bb.get(freq_band_4);
	bb.setOffset(37);	bb.get(channel_4);
	bb.setOffset(38);	carrier_noise_4 = static_cast<float>(bb.get<uint16_t>()) * (0.003);
	bb.setOffset(40);	bb.get(freq_band_5);
	bb.setOffset(41);	bb.get(channel_5);
	bb.setOffset(42);	carrier_noise_5 = static_cast<float>(bb.get<uint16_t>()) * (0.003);
	bb.setOffset(44);	bb.get(freq_band_6);
	bb.setOffset(45);	bb.get(channel_6);
	bb.setOffset(46);	carrier_noise_6 = static_cast<float>(bb.get<uint16_t>()) * (0.003);

	uint32_t InsGnssSummary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(48);	bb.get(InsGnssSummary_tmp);
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
	bb.setOffset(52);	bb.get(gps_week);
	bb.setOffset(56);	bb.get(gpsTov);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 16);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

