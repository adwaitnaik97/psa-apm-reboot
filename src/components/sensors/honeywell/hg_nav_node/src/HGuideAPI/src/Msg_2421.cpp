#include <include/HGuideAPI.h>
#include <include/Msg_2421.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2421::AddressId;
const uint32_t Msg_2421::MessageId;
const uint32_t Msg_2421::MessageLength;

Msg_2421::Msg_2421()
{
	Default();
}

void Msg_2421::Default()
{
	Checksum = 0;
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	gps_pr_aiding_status.Default();
	gps_dr_aiding_status.Default();
	ta_aiding_status.Default();
	gnss_pvt_aiding_status.Default();
	misc_aiding_status.Default();
	Available_Measurements = 0;
	Successfully_Processed = 0;
	Failed_Processing = 0;
}

bool Msg_2421::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 124) return false;

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

	uint32_t gps_pr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 0, 1, gps_pr_aiding_status.channel_1, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 2, 3, gps_pr_aiding_status.channel_2, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 4, 5, gps_pr_aiding_status.channel_3, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 6, 7, gps_pr_aiding_status.channel_4, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 8, 9, gps_pr_aiding_status.channel_5, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 10, 11, gps_pr_aiding_status.channel_6, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 12, 13, gps_pr_aiding_status.channel_7, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 14, 15, gps_pr_aiding_status.channel_8, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 16, 17, gps_pr_aiding_status.channel_9, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 18, 19, gps_pr_aiding_status.channel_10, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 20, 21, gps_pr_aiding_status.channel_11, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 22, 23, gps_pr_aiding_status.channel_12, status_ok);
	gps_pr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_pr_aiding_status_tmp, 31, 31, gps_pr_aiding_status.TimeBiasRepartitionEvent, status_ok);
	bb.setOffset(36);	bb.put(gps_pr_aiding_status_tmp);


	uint32_t gps_dr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 0, 1, gps_dr_aiding_status.channel_1, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 2, 3, gps_dr_aiding_status.channel_2, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 4, 5, gps_dr_aiding_status.channel_3, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 6, 7, gps_dr_aiding_status.channel_4, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 8, 9, gps_dr_aiding_status.channel_5, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 10, 11, gps_dr_aiding_status.channel_6, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 12, 13, gps_dr_aiding_status.channel_7, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 14, 15, gps_dr_aiding_status.channel_8, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 16, 17, gps_dr_aiding_status.channel_9, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 18, 19, gps_dr_aiding_status.channel_10, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 20, 21, gps_dr_aiding_status.channel_11, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 22, 23, gps_dr_aiding_status.channel_12, status_ok);
	gps_dr_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gps_dr_aiding_status_tmp, 31, 31, gps_dr_aiding_status.TimeBiasRepartitionEvent, status_ok);
	bb.setOffset(44);	bb.put(gps_dr_aiding_status_tmp);


	uint32_t ta_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 0, 1, ta_aiding_status.position_ecef_X, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 2, 3, ta_aiding_status.position_ecef_Y, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 4, 5, ta_aiding_status.position_ecef_Z, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 8, 9, ta_aiding_status.velocity_ecef_X, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 10, 11, ta_aiding_status.velocity_ecef_Y, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 12, 13, ta_aiding_status.velocity_ecef_Z, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 16, 17, ta_aiding_status.attitude_roll, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 18, 19, ta_aiding_status.attitude_pitch, status_ok);
	ta_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(ta_aiding_status_tmp, 20, 21, ta_aiding_status.attitude_heading, status_ok);
	bb.setOffset(52);	bb.put(ta_aiding_status_tmp);


	uint32_t gnss_pvt_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 0, 1, gnss_pvt_aiding_status.latitude, status_ok);
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 2, 3, gnss_pvt_aiding_status.longitude, status_ok);
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 4, 5, gnss_pvt_aiding_status.altitude, status_ok);
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 8, 9, gnss_pvt_aiding_status.velocity_north, status_ok);
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 10, 11, gnss_pvt_aiding_status.velocity_east, status_ok);
	gnss_pvt_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(gnss_pvt_aiding_status_tmp, 12, 13, gnss_pvt_aiding_status.velocity_down, status_ok);
	bb.setOffset(56);	bb.put(gnss_pvt_aiding_status_tmp);


	uint32_t misc_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 0, 1, misc_aiding_status.zero_velocity_x, status_ok);
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 2, 3, misc_aiding_status.zero_velocity_y, status_ok);
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 4, 5, misc_aiding_status.zero_velocity_z, status_ok);
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 8, 9, misc_aiding_status.magnetic_heading, status_ok);
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 16, 17, misc_aiding_status.zero_heading_change, status_ok);
	misc_aiding_status_tmp = ECTOS::BIT_UTILITIES::Pack(misc_aiding_status_tmp, 18, 19, misc_aiding_status.barometric_altitude, status_ok);
	bb.setOffset(60);	bb.put(misc_aiding_status_tmp);

	bb.setOffset(64);	bb.put(Available_Measurements);
	bb.setOffset(68);	bb.put(Successfully_Processed);
	bb.setOffset(72);	bb.put(Failed_Processing);
	Checksum = computeChecksum((uint32_t*)buffer, 31);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2421::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 124) return -2;

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

	uint32_t gps_pr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(36);	bb.get(gps_pr_aiding_status_tmp);
	gps_pr_aiding_status.channel_1 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 0, 1,  status_ok);
	gps_pr_aiding_status.channel_2 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 2, 3,  status_ok);
	gps_pr_aiding_status.channel_3 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 4, 5,  status_ok);
	gps_pr_aiding_status.channel_4 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 6, 7,  status_ok);
	gps_pr_aiding_status.channel_5 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 8, 9,  status_ok);
	gps_pr_aiding_status.channel_6 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 10, 11,  status_ok);
	gps_pr_aiding_status.channel_7 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 12, 13,  status_ok);
	gps_pr_aiding_status.channel_8 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 14, 15,  status_ok);
	gps_pr_aiding_status.channel_9 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 16, 17,  status_ok);
	gps_pr_aiding_status.channel_10 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 18, 19,  status_ok);
	gps_pr_aiding_status.channel_11 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 20, 21,  status_ok);
	gps_pr_aiding_status.channel_12 = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 22, 23,  status_ok);
	gps_pr_aiding_status.TimeBiasRepartitionEvent = ECTOS::BIT_UTILITIES::UnPack(gps_pr_aiding_status_tmp, 31, 31,  status_ok);

	uint32_t gps_dr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(44);	bb.get(gps_dr_aiding_status_tmp);
	gps_dr_aiding_status.channel_1 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 0, 1,  status_ok);
	gps_dr_aiding_status.channel_2 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 2, 3,  status_ok);
	gps_dr_aiding_status.channel_3 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 4, 5,  status_ok);
	gps_dr_aiding_status.channel_4 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 6, 7,  status_ok);
	gps_dr_aiding_status.channel_5 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 8, 9,  status_ok);
	gps_dr_aiding_status.channel_6 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 10, 11,  status_ok);
	gps_dr_aiding_status.channel_7 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 12, 13,  status_ok);
	gps_dr_aiding_status.channel_8 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 14, 15,  status_ok);
	gps_dr_aiding_status.channel_9 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 16, 17,  status_ok);
	gps_dr_aiding_status.channel_10 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 18, 19,  status_ok);
	gps_dr_aiding_status.channel_11 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 20, 21,  status_ok);
	gps_dr_aiding_status.channel_12 = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 22, 23,  status_ok);
	gps_dr_aiding_status.TimeBiasRepartitionEvent = ECTOS::BIT_UTILITIES::UnPack(gps_dr_aiding_status_tmp, 31, 31,  status_ok);

	uint32_t ta_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(52);	bb.get(ta_aiding_status_tmp);
	ta_aiding_status.position_ecef_X = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 0, 1,  status_ok);
	ta_aiding_status.position_ecef_Y = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 2, 3,  status_ok);
	ta_aiding_status.position_ecef_Z = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 4, 5,  status_ok);
	ta_aiding_status.velocity_ecef_X = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 8, 9,  status_ok);
	ta_aiding_status.velocity_ecef_Y = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 10, 11,  status_ok);
	ta_aiding_status.velocity_ecef_Z = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 12, 13,  status_ok);
	ta_aiding_status.attitude_roll = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 16, 17,  status_ok);
	ta_aiding_status.attitude_pitch = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 18, 19,  status_ok);
	ta_aiding_status.attitude_heading = ECTOS::BIT_UTILITIES::UnPack(ta_aiding_status_tmp, 20, 21,  status_ok);

	uint32_t gnss_pvt_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(56);	bb.get(gnss_pvt_aiding_status_tmp);
	gnss_pvt_aiding_status.latitude = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 0, 1,  status_ok);
	gnss_pvt_aiding_status.longitude = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 2, 3,  status_ok);
	gnss_pvt_aiding_status.altitude = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 4, 5,  status_ok);
	gnss_pvt_aiding_status.velocity_north = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 8, 9,  status_ok);
	gnss_pvt_aiding_status.velocity_east = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 10, 11,  status_ok);
	gnss_pvt_aiding_status.velocity_down = ECTOS::BIT_UTILITIES::UnPack(gnss_pvt_aiding_status_tmp, 12, 13,  status_ok);

	uint32_t misc_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(60);	bb.get(misc_aiding_status_tmp);
	misc_aiding_status.zero_velocity_x = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 0, 1,  status_ok);
	misc_aiding_status.zero_velocity_y = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 2, 3,  status_ok);
	misc_aiding_status.zero_velocity_z = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 4, 5,  status_ok);
	misc_aiding_status.magnetic_heading = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 8, 9,  status_ok);
	misc_aiding_status.zero_heading_change = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 16, 17,  status_ok);
	misc_aiding_status.barometric_altitude = ECTOS::BIT_UTILITIES::UnPack(misc_aiding_status_tmp, 18, 19,  status_ok);
	bb.setOffset(64);	bb.get(Available_Measurements);
	bb.setOffset(68);	bb.get(Successfully_Processed);
	bb.setOffset(72);	bb.get(Failed_Processing);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 31);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

