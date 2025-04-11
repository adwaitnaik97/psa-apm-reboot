#include <include/HGuideAPI.h>
#include <include/Msg_1001.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1001::AddressId;
const uint32_t Msg_1001::MessageId;
const uint32_t Msg_1001::MessageLength;

Msg_1001::Msg_1001()
{
	Default();
}

void Msg_1001::Default()
{
	Checksum = 0;
	MessageWord1.Default();
	MessageWord2.Default();
	MessageWord3.Default();
	MessageWord4.Default();
}

bool Msg_1001::Serialize(unsigned char * buffer, const unsigned int bufferSize)
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

	uint32_t MessageWord1_tmp = 0; // temporary variable holding the custom bitfield
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 0, MessageWord1.INS_CONFIG_2001, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 1, MessageWord1.INS_MODE_STATUS_2011_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 2, MessageWord1.TIMEMARK_EVENT_IN_6201, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 3, MessageWord1.INS_INIT_STATUS_2021_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 4, MessageWord1.SKYMAP_DATA_6505, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 5, MessageWord1.GPS_PVT_OUT_2108_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 6, MessageWord1.EVENT_IN_GEODETIC_POSITION_6202, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 7, MessageWord1.EVENT_IN_NED_VELOCITY_6203, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 8, MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 9, MessageWord1.TIMEMARK_BLOCK4_2211_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 10, MessageWord1.EVENT_IN_EULER_ATTITUDE_6204, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 11, MessageWord1.EVENT_IN_FULL_6205, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 12, MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 13, MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 14, MessageWord1.UNFILTERED_INS_DATA_2311_NC, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 15, MessageWord1.SAVE_CONFIGURATION_TO_FLASH, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 16, MessageWord1.NAV_OUT_2401_100HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 17, MessageWord1.NAV_OUT_2401_50HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 18, MessageWord1.NAV_OUT_2401_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 19, MessageWord1.SMOOTH_NAV_OUT_2402_100HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 20, MessageWord1.SMOOTH_NAV_OUT_2402_50HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 21, MessageWord1.SMOOTH_NAV_OUT_2402_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 22, MessageWord1.INS_ERR_EST_2411_5HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 23, MessageWord1.INS_ERR_EST_2411_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 24, MessageWord1.KF_NAV_MEAS_ST_2421_5HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 25, MessageWord1.KF_NAV_MEAS_ST_2421_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 26, MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 27, MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 28, MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 29, MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 30, MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ, status_ok);
	MessageWord1_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord1_tmp, 31, MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ, status_ok);
	bb.setOffset(16);	bb.put(MessageWord1_tmp);


	uint32_t MessageWord2_tmp = 0; // temporary variable holding the custom bitfield
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 0, MessageWord2.GPS_CONFIG_2002, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 1, MessageWord2.MOTION_DETECT_6111, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 2, MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 3, MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 4, MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 5, MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 6, MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 7, MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 8, MessageWord2.GPS_CHANNEL_STATUS_2501_SET, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 9, MessageWord2.GPS_ALMANAC_2511_SET, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 10, MessageWord2.GPS_EPHEMERIS_2512_SET, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 11, MessageWord2.GPS_ONAV_2513_SET, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 12, MessageWord2.GPS_IONO_2514_SET, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 13, MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 14, MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 15, MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 16, MessageWord2.IBIT_RESULTS_2601, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 17, MessageWord2.BIT_HISTORY_2611, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 18, MessageWord2.GNSS_PVT_6108, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 19, MessageWord2.GEODETIC_POSITION_6403, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 20, MessageWord2.EULER_ATTITUDE_6405, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 21, MessageWord2.NED_VELOCITY_6504, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 22, MessageWord2.GNSS_ATT_6109, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 23, MessageWord2.ODO_DPOS_6110, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 24, MessageWord2.APP_SP_OUTPUT_1_2901, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 25, MessageWord2.GNSS_SINGLE_ANT_ATT_6112, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 26, MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 27, MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 28, MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 29, MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 30, MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ, status_ok);
	MessageWord2_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord2_tmp, 31, MessageWord2.LOG_MESSAGE_9900, status_ok);
	bb.setOffset(20);	bb.put(MessageWord2_tmp);


	uint32_t MessageWord3_tmp = 0; // temporary variable holding the custom bitfield
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 0, MessageWord3.LED_STATUS_MESSAGE_6601, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 1, MessageWord3.HGUIDE_INSTALL_6003, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 2, MessageWord3.ANTENNA_CONNECTED_6508, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 3, MessageWord3.DEVICE_IDENTIFICATION_6001, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 4, MessageWord3.MESSAGE_WORD_3_BIT_4, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 5, MessageWord3.MESSAGE_WORD_3_BIT_5, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 6, MessageWord3.MESSAGE_WORD_3_BIT_6, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 7, MessageWord3.MESSAGE_WORD_3_BIT_7, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 8, MessageWord3.MESSAGE_WORD_3_BIT_8, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 9, MessageWord3.REAL_TIME_GOOD_TO_GO_6651, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 10, MessageWord3.MESSAGE_WORD_3_BIT_10, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 11, MessageWord3.MESSAGE_WORD_3_BIT_11, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 12, MessageWord3.MESSAGE_WORD_3_BIT_12, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 13, MessageWord3.MESSAGE_WORD_3_BIT_13, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 14, MessageWord3.MESSAGE_WORD_3_BIT_14, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 15, MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 16, MessageWord3.NORTEK_DVL_WATER_TRACK_6722, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 17, MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 18, MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 19, MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 20, MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 21, MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 22, MessageWord3.MESSAGE_WORD_3_BIT_22, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 23, MessageWord3.MESSAGE_WORD_3_BIT_23, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 24, MessageWord3.MESSAGE_WORD_3_BIT_24, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 25, MessageWord3.MESSAGE_WORD_3_BIT_25, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 26, MessageWord3.MESSAGE_WORD_3_BIT_26, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 27, MessageWord3.MESSAGE_WORD_3_BIT_27, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 28, MessageWord3.MESSAGE_WORD_3_BIT_28, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 29, MessageWord3.MESSAGE_WORD_3_BIT_29, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 30, MessageWord3.MESSAGE_WORD_3_BIT_30, status_ok);
	MessageWord3_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord3_tmp, 31, MessageWord3.PROCESS_LOAD_9910, status_ok);
	bb.setOffset(24);	bb.put(MessageWord3_tmp);


	uint32_t MessageWord4_tmp = 0; // temporary variable holding the custom bitfield
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 0, MessageWord4.GNSS_RECEIVER_VERSION_5001, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 1, MessageWord4.MESSAGE_WORD_4_BIT_2, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 2, MessageWord4.MESSAGE_WORD_4_BIT_3, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 3, MessageWord4.GNSS_RECEIVER_STATUS_5012, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 4, MessageWord4.MESSAGE_WORD_4_BIT_4, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 5, MessageWord4.GNSS_RANGE_5101_1Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 6, MessageWord4.GNSS_RANGE_5101_5Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 7, MessageWord4.GNSS_RANGE_5101_10Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 8, MessageWord4.MESSAGE_WORD_4_BIT_8, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 9, MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 10, MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 11, MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 12, MessageWord4.MESSAGE_WORD_4_BIT_12, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 13, MessageWord4.GPS_EPHEMERIS_5103, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 14, MessageWord4.GLONASS_EPHEMERIS_5104, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 15, MessageWord4.GALILEO_EPHEMERIS_5105, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 16, MessageWord4.BEIDOU_EPHEMERIS_5106, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 17, MessageWord4.MESSAGE_WORD_4_BIT_17, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 18, MessageWord4.MESSAGE_WORD_4_BIT_18, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 19, MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 20, MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 21, MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 22, MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 23, MessageWord4.MESSAGE_WORD_4_BIT_23, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 24, MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 25, MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 26, MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 27, MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 28, MessageWord4.MESSAGE_WORD_4_BIT_28, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 29, MessageWord4.MESSAGE_WORD_4_BIT_29, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 30, MessageWord4.GNSS_PPS_TIME_5201, status_ok);
	MessageWord4_tmp = ECTOS::BIT_UTILITIES::PackBool(MessageWord4_tmp, 31, MessageWord4.MESSAGE_WORD_4_BIT_31, status_ok);
	bb.setOffset(28);	bb.put(MessageWord4_tmp);

	Checksum = computeChecksum((uint32_t*)buffer, 16);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1001::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
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

	uint32_t MessageWord1_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(16);	bb.get(MessageWord1_tmp);
	MessageWord1.INS_CONFIG_2001 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 0,  status_ok);
	MessageWord1.INS_MODE_STATUS_2011_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 1,  status_ok);
	MessageWord1.TIMEMARK_EVENT_IN_6201 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 2,  status_ok);
	MessageWord1.INS_INIT_STATUS_2021_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 3,  status_ok);
	MessageWord1.SKYMAP_DATA_6505 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 4,  status_ok);
	MessageWord1.GPS_PVT_OUT_2108_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 5,  status_ok);
	MessageWord1.EVENT_IN_GEODETIC_POSITION_6202 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 6,  status_ok);
	MessageWord1.EVENT_IN_NED_VELOCITY_6203 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 7,  status_ok);
	MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 8,  status_ok);
	MessageWord1.TIMEMARK_BLOCK4_2211_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 9,  status_ok);
	MessageWord1.EVENT_IN_EULER_ATTITUDE_6204 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 10,  status_ok);
	MessageWord1.EVENT_IN_FULL_6205 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 11,  status_ok);
	MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 12,  status_ok);
	MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 13,  status_ok);
	MessageWord1.UNFILTERED_INS_DATA_2311_NC = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 14,  status_ok);
	MessageWord1.SAVE_CONFIGURATION_TO_FLASH = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 15,  status_ok);
	MessageWord1.NAV_OUT_2401_100HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 16,  status_ok);
	MessageWord1.NAV_OUT_2401_50HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 17,  status_ok);
	MessageWord1.NAV_OUT_2401_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 18,  status_ok);
	MessageWord1.SMOOTH_NAV_OUT_2402_100HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 19,  status_ok);
	MessageWord1.SMOOTH_NAV_OUT_2402_50HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 20,  status_ok);
	MessageWord1.SMOOTH_NAV_OUT_2402_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 21,  status_ok);
	MessageWord1.INS_ERR_EST_2411_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 22,  status_ok);
	MessageWord1.INS_ERR_EST_2411_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 23,  status_ok);
	MessageWord1.KF_NAV_MEAS_ST_2421_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 24,  status_ok);
	MessageWord1.KF_NAV_MEAS_ST_2421_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 25,  status_ok);
	MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 26,  status_ok);
	MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 27,  status_ok);
	MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 28,  status_ok);
	MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 29,  status_ok);
	MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 30,  status_ok);
	MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord1_tmp, 31,  status_ok);

	uint32_t MessageWord2_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(20);	bb.get(MessageWord2_tmp);
	MessageWord2.GPS_CONFIG_2002 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 0,  status_ok);
	MessageWord2.MOTION_DETECT_6111 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 1,  status_ok);
	MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 2,  status_ok);
	MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 3,  status_ok);
	MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 4,  status_ok);
	MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 5,  status_ok);
	MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 6,  status_ok);
	MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 7,  status_ok);
	MessageWord2.GPS_CHANNEL_STATUS_2501_SET = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 8,  status_ok);
	MessageWord2.GPS_ALMANAC_2511_SET = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 9,  status_ok);
	MessageWord2.GPS_EPHEMERIS_2512_SET = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 10,  status_ok);
	MessageWord2.GPS_ONAV_2513_SET = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 11,  status_ok);
	MessageWord2.GPS_IONO_2514_SET = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 12,  status_ok);
	MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 13,  status_ok);
	MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 14,  status_ok);
	MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 15,  status_ok);
	MessageWord2.IBIT_RESULTS_2601 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 16,  status_ok);
	MessageWord2.BIT_HISTORY_2611 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 17,  status_ok);
	MessageWord2.GNSS_PVT_6108 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 18,  status_ok);
	MessageWord2.GEODETIC_POSITION_6403 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 19,  status_ok);
	MessageWord2.EULER_ATTITUDE_6405 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 20,  status_ok);
	MessageWord2.NED_VELOCITY_6504 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 21,  status_ok);
	MessageWord2.GNSS_ATT_6109 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 22,  status_ok);
	MessageWord2.ODO_DPOS_6110 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 23,  status_ok);
	MessageWord2.APP_SP_OUTPUT_1_2901 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 24,  status_ok);
	MessageWord2.GNSS_SINGLE_ANT_ATT_6112 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 25,  status_ok);
	MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 26,  status_ok);
	MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 27,  status_ok);
	MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 28,  status_ok);
	MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 29,  status_ok);
	MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 30,  status_ok);
	MessageWord2.LOG_MESSAGE_9900 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord2_tmp, 31,  status_ok);

	uint32_t MessageWord3_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(24);	bb.get(MessageWord3_tmp);
	MessageWord3.LED_STATUS_MESSAGE_6601 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 0,  status_ok);
	MessageWord3.HGUIDE_INSTALL_6003 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 1,  status_ok);
	MessageWord3.ANTENNA_CONNECTED_6508 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 2,  status_ok);
	MessageWord3.DEVICE_IDENTIFICATION_6001 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 3,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_4 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 4,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_5 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 5,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_6 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 6,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_7 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 7,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_8 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 8,  status_ok);
	MessageWord3.REAL_TIME_GOOD_TO_GO_6651 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 9,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_10 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 10,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_11 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 11,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_12 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 12,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_13 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 13,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_14 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 14,  status_ok);
	MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 15,  status_ok);
	MessageWord3.NORTEK_DVL_WATER_TRACK_6722 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 16,  status_ok);
	MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 17,  status_ok);
	MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 18,  status_ok);
	MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 19,  status_ok);
	MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 20,  status_ok);
	MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 21,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_22 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 22,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_23 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 23,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_24 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 24,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_25 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 25,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_26 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 26,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_27 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 27,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_28 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 28,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_29 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 29,  status_ok);
	MessageWord3.MESSAGE_WORD_3_BIT_30 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 30,  status_ok);
	MessageWord3.PROCESS_LOAD_9910 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord3_tmp, 31,  status_ok);

	uint32_t MessageWord4_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(28);	bb.get(MessageWord4_tmp);
	MessageWord4.GNSS_RECEIVER_VERSION_5001 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 0,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_2 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 1,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_3 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 2,  status_ok);
	MessageWord4.GNSS_RECEIVER_STATUS_5012 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 3,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_4 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 4,  status_ok);
	MessageWord4.GNSS_RANGE_5101_1Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 5,  status_ok);
	MessageWord4.GNSS_RANGE_5101_5Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 6,  status_ok);
	MessageWord4.GNSS_RANGE_5101_10Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 7,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_8 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 8,  status_ok);
	MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 9,  status_ok);
	MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 10,  status_ok);
	MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 11,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_12 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 12,  status_ok);
	MessageWord4.GPS_EPHEMERIS_5103 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 13,  status_ok);
	MessageWord4.GLONASS_EPHEMERIS_5104 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 14,  status_ok);
	MessageWord4.GALILEO_EPHEMERIS_5105 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 15,  status_ok);
	MessageWord4.BEIDOU_EPHEMERIS_5106 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 16,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_17 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 17,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_18 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 18,  status_ok);
	MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 19,  status_ok);
	MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 20,  status_ok);
	MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 21,  status_ok);
	MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 22,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_23 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 23,  status_ok);
	MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 24,  status_ok);
	MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 25,  status_ok);
	MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 26,  status_ok);
	MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 27,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_28 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 28,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_29 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 29,  status_ok);
	MessageWord4.GNSS_PPS_TIME_5201 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 30,  status_ok);
	MessageWord4.MESSAGE_WORD_4_BIT_31 = ECTOS::BIT_UTILITIES::UnPackBool(MessageWord4_tmp, 31,  status_ok);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 16);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

