#include <include/HGuideAPI.h>
#include <include/Msg_2011.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_2011::AddressId;
const uint32_t Msg_2011::MessageId;
const uint32_t Msg_2011::MessageLength;

Msg_2011::Msg_2011()
{
	Default();
}

void Msg_2011::Default()
{
	Checksum = 0;
	inertial_data_source = 0;
	navigation_initialization_status = 0;
	navErrorModel = 0;
	INSMode = 0;
	InsGnssSummary.Default();
	systemTov = 0;
	gpsTov = 0;
	gps_week = 0;
	power_cycle_count = 0;
	eti = 0;
	ins_device_temperature = 0;
	bitMode = 0;
	ins_blended_fom = 0;
	gps_fom = 0;
	utc_tfom = 0;
	MDT_stationary_instantaneous_signal = 0;
	MDT_stationary_accel_mag = 0;
	MDT_stationary_speed_stdv = 0;
	MDT_stationary_rotation_rate = 0;
	MessageWord1.Default();
	MessageWord2.Default();
	imu_bit_status.Default();
	aiding_sources.Default();
	number_of_satellites_in_blended_sol = 0;
	pr_validity_by_gps_receiver_channel.Default();
	dr_validity_by_gps_receiver_channel.Default();
	ssbl_status.Default();
	solution_convergence = 0;
	attitude_fom = 0;
	gnssr_aiding_status.Default();
	fsbl_status.Default();
	system_bit_summary.Default();
	mcb_bit_status.Default();
	gps_bit_status.Default();
	gyro_bit_status.Default();
	accelerometer_bit_status.Default();
	accel_x_temperature_degC = 0;
	accel_y_temperature_degC = 0;
	accel_z_temperature_degC = 0;
	gyro_x_temperature_degC = 0;
	gyro_y_temperature_degC = 0;
	gyro_z_temperature_degC = 0;
	MDTC_zero_vel_meas_noiseX = 0;
	MDTC_inv_nominal_signal_filter_fn = 0;
	MDTC_instant_signal_filter_fn = 0;
	MDTC_settling_time = 0;
}

bool Msg_2011::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 220) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(inertial_data_source);
	bb.setOffset(21);	bb.put(navigation_initialization_status);
	bb.setOffset(22);	bb.put(navErrorModel);
	bb.setOffset(23);	bb.put(INSMode);

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
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 17, 19, InsGnssSummary.SnapbackStatus, status_ok);
	InsGnssSummary_tmp = ECTOS::BIT_UTILITIES::Pack(InsGnssSummary_tmp, 28, 31, InsGnssSummary.GPSMode, status_ok);
	bb.setOffset(28);	bb.put(InsGnssSummary_tmp);

	bb.setOffset(32);	bb.put(systemTov);
	bb.setOffset(40);	bb.put(gpsTov);
	bb.setOffset(48);	bb.put(gps_week);
	bb.setOffset(52);	bb.put(power_cycle_count);
	bb.setOffset(56);	bb.put(eti);
	bb.setOffset(64);	bb.put(ins_device_temperature);
	bb.setOffset(68);	bb.put(bitMode);
	bb.setOffset(72);	bb.put(ins_blended_fom);
	bb.setOffset(76);	bb.put(gps_fom);
	bb.setOffset(80);	bb.put(utc_tfom);
	bb.setOffset(88);	bb.put(MDT_stationary_instantaneous_signal);
	bb.setOffset(89);	bb.put(MDT_stationary_accel_mag);
	bb.setOffset(90);	bb.put(MDT_stationary_speed_stdv);
	bb.setOffset(91);	bb.put(MDT_stationary_rotation_rate);

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
	bb.setOffset(92);	bb.put(MessageWord1_tmp);


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
	bb.setOffset(96);	bb.put(MessageWord2_tmp);


	uint32_t imu_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 0, imu_bit_status.imu_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 1, imu_bit_status.processor_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 2, imu_bit_status.memory_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 3, imu_bit_status.other_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 4, imu_bit_status.accelerometer_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 5, imu_bit_status.gyro_failed, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 6, imu_bit_status.external_sync_fault, status_ok);
	imu_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(imu_bit_status_tmp, 28, imu_bit_status.imu_bit_mode, status_ok);
	bb.setOffset(100);	bb.put(imu_bit_status_tmp);


	uint32_t aiding_sources_tmp = 0; // temporary variable holding the custom bitfield
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 0, aiding_sources.zero_velocity, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 1, aiding_sources.GNSS_PVT_P, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 2, aiding_sources.GNSS_PVT_V, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 3, aiding_sources.GNSS_ATT_A, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 4, aiding_sources.GNSS_pseudo_range, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 5, aiding_sources.GNSS_delta_range, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 7, aiding_sources.baro_altitude, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 8, aiding_sources.position_match, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 9, aiding_sources.velocity_match, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 10, aiding_sources.attitude_match, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 11, aiding_sources.mag_heading, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 12, aiding_sources.static_heading, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 13, aiding_sources.motion_detect, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 15, aiding_sources.odometer, status_ok);
	aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(aiding_sources_tmp, 16, aiding_sources.land_constraints, status_ok);
	bb.setOffset(104);	bb.put(aiding_sources_tmp);

	bb.setOffset(112);	bb.put(number_of_satellites_in_blended_sol);

	uint32_t pr_validity_by_gps_receiver_channel_tmp = 0; // temporary variable holding the custom bitfield
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 0, pr_validity_by_gps_receiver_channel.channel_0, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 1, pr_validity_by_gps_receiver_channel.channel_1, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 2, pr_validity_by_gps_receiver_channel.channel_2, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 3, pr_validity_by_gps_receiver_channel.channel_3, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 4, pr_validity_by_gps_receiver_channel.channel_4, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 5, pr_validity_by_gps_receiver_channel.channel_5, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 6, pr_validity_by_gps_receiver_channel.channel_6, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 7, pr_validity_by_gps_receiver_channel.channel_7, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 8, pr_validity_by_gps_receiver_channel.channel_8, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 9, pr_validity_by_gps_receiver_channel.channel_9, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 10, pr_validity_by_gps_receiver_channel.channel_10, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 11, pr_validity_by_gps_receiver_channel.channel_11, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 12, pr_validity_by_gps_receiver_channel.channel_12, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 13, pr_validity_by_gps_receiver_channel.channel_13, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 14, pr_validity_by_gps_receiver_channel.channel_14, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 15, pr_validity_by_gps_receiver_channel.channel_15, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 16, pr_validity_by_gps_receiver_channel.channel_16, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 17, pr_validity_by_gps_receiver_channel.channel_17, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 18, pr_validity_by_gps_receiver_channel.channel_18, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 19, pr_validity_by_gps_receiver_channel.channel_19, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 20, pr_validity_by_gps_receiver_channel.channel_20, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 21, pr_validity_by_gps_receiver_channel.channel_21, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 22, pr_validity_by_gps_receiver_channel.channel_22, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 23, pr_validity_by_gps_receiver_channel.channel_23, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 24, pr_validity_by_gps_receiver_channel.channel_24, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 25, pr_validity_by_gps_receiver_channel.channel_25, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 26, pr_validity_by_gps_receiver_channel.channel_26, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 27, pr_validity_by_gps_receiver_channel.channel_27, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 28, pr_validity_by_gps_receiver_channel.channel_28, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 29, pr_validity_by_gps_receiver_channel.channel_29, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 30, pr_validity_by_gps_receiver_channel.channel_30, status_ok);
	pr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(pr_validity_by_gps_receiver_channel_tmp, 31, pr_validity_by_gps_receiver_channel.channel_31, status_ok);
	bb.setOffset(116);	bb.put(pr_validity_by_gps_receiver_channel_tmp);


	uint32_t dr_validity_by_gps_receiver_channel_tmp = 0; // temporary variable holding the custom bitfield
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 0, dr_validity_by_gps_receiver_channel.channel_0, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 1, dr_validity_by_gps_receiver_channel.channel_1, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 2, dr_validity_by_gps_receiver_channel.channel_2, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 3, dr_validity_by_gps_receiver_channel.channel_3, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 4, dr_validity_by_gps_receiver_channel.channel_4, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 5, dr_validity_by_gps_receiver_channel.channel_5, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 6, dr_validity_by_gps_receiver_channel.channel_6, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 7, dr_validity_by_gps_receiver_channel.channel_7, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 8, dr_validity_by_gps_receiver_channel.channel_8, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 9, dr_validity_by_gps_receiver_channel.channel_9, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 10, dr_validity_by_gps_receiver_channel.channel_10, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 11, dr_validity_by_gps_receiver_channel.channel_11, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 12, dr_validity_by_gps_receiver_channel.channel_12, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 13, dr_validity_by_gps_receiver_channel.channel_13, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 14, dr_validity_by_gps_receiver_channel.channel_14, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 15, dr_validity_by_gps_receiver_channel.channel_15, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 16, dr_validity_by_gps_receiver_channel.channel_16, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 17, dr_validity_by_gps_receiver_channel.channel_17, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 18, dr_validity_by_gps_receiver_channel.channel_18, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 19, dr_validity_by_gps_receiver_channel.channel_19, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 20, dr_validity_by_gps_receiver_channel.channel_20, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 21, dr_validity_by_gps_receiver_channel.channel_21, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 22, dr_validity_by_gps_receiver_channel.channel_22, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 23, dr_validity_by_gps_receiver_channel.channel_23, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 24, dr_validity_by_gps_receiver_channel.channel_24, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 25, dr_validity_by_gps_receiver_channel.channel_25, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 26, dr_validity_by_gps_receiver_channel.channel_26, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 27, dr_validity_by_gps_receiver_channel.channel_27, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 28, dr_validity_by_gps_receiver_channel.channel_28, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 29, dr_validity_by_gps_receiver_channel.channel_29, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 30, dr_validity_by_gps_receiver_channel.channel_30, status_ok);
	dr_validity_by_gps_receiver_channel_tmp = ECTOS::BIT_UTILITIES::PackBool(dr_validity_by_gps_receiver_channel_tmp, 31, dr_validity_by_gps_receiver_channel.channel_31, status_ok);
	bb.setOffset(120);	bb.put(dr_validity_by_gps_receiver_channel_tmp);


	uint32_t ssbl_status_tmp = 0; // temporary variable holding the custom bitfield
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 0, ssbl_status.flash_register_init_table_missing, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 1, ssbl_status.flash_register_init_file_checksum_error, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 2, ssbl_status.flash_table_engry_checksum_error, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 3, ssbl_status.flash_table_entry_address_error, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 4, ssbl_status.ssbl_register_init_table_missing, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 5, ssbl_status.ssbl_register_init_file_checksum_error, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 6, ssbl_status.ssbl_table_engry_checksum_error, status_ok);
	ssbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(ssbl_status_tmp, 7, ssbl_status.ssbl_table_entry_address_error, status_ok);
	bb.setOffset(124);	bb.put(ssbl_status_tmp);

	bb.setOffset(128);	bb.put(solution_convergence);
	bb.setOffset(132);	bb.put(attitude_fom);

	uint32_t gnssr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 0, gnssr_aiding_status.gnssr_receiver_inertial_aiding_capable, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 1, gnssr_aiding_status.gnssr_inertial_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 2, gnssr_aiding_status.gnssr_inertial_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 3, gnssr_aiding_status.gnssr_drs_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 4, gnssr_aiding_status.gnssr_drs_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 5, gnssr_aiding_status.gnssr_baro_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 6, gnssr_aiding_status.gnssr_baro_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 7, gnssr_aiding_status.gnssr_tas_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 8, gnssr_aiding_status.gnssr_tas_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 9, gnssr_aiding_status.gnssr_mag_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 10, gnssr_aiding_status.gnssr_mag_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 11, gnssr_aiding_status.gnssr_attitude_aiding_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 12, gnssr_aiding_status.gnssr_attitude_aiding_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 13, gnssr_aiding_status.gnssr_std_frequency_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 14, gnssr_aiding_status.gnssr_std_frequency_in_use, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 15, gnssr_aiding_status.gnssr_std_time_valid, status_ok);
	gnssr_aiding_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gnssr_aiding_status_tmp, 16, gnssr_aiding_status.gnssr_std_time_in_use, status_ok);
	bb.setOffset(136);	bb.put(gnssr_aiding_status_tmp);


	uint64_t fsbl_status_tmp = 0; // temporary variable holding the custom bitfield
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::Pack(fsbl_status_tmp, 0, 15, fsbl_status.fsbl_boot_rom_error_code, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 16, fsbl_status.fslb_system_wdt_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 17, fsbl_status.fsbl_apu_0_wdt_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 18, fsbl_status.fsbl_apu_1_wdt_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 19, fsbl_status.fsbl_software_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 20, fsbl_status.fsbl_debug_system_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 21, fsbl_status.fsbl_external_system_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 22, fsbl_status.fsbl_power_on_reset, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::Pack(fsbl_status_tmp, 24, 31, fsbl_status.fsbl_reboot_state, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 32, fsbl_status.fsbl_pmic_boost_status, status_ok);
	fsbl_status_tmp = ECTOS::BIT_UTILITIES::PackBool(fsbl_status_tmp, 33, fsbl_status.fsbl_gpga_initialization_done, status_ok);
	bb.setOffset(140);	bb.put(fsbl_status_tmp);


	uint32_t system_bit_summary_tmp = 0; // temporary variable holding the custom bitfield
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 0, system_bit_summary.Go_NoGo_BIT_Summary, status_ok);
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 1, system_bit_summary.MPB_Subsystem_BIT_Summary, status_ok);
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 2, system_bit_summary.GPS_Subsystem_BIT_Summary, status_ok);
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 3, system_bit_summary.Gyro_Subsystem_BIT_Summary, status_ok);
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 4, system_bit_summary.Accelerometer_Subsystem_BIT_Summary, status_ok);
	system_bit_summary_tmp = ECTOS::BIT_UTILITIES::PackBool(system_bit_summary_tmp, 6, system_bit_summary.IMU_Subsystem_BIT_Summary, status_ok);
	bb.setOffset(148);	bb.put(system_bit_summary_tmp);


	uint64_t mcb_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 0, mcb_bit_status.mcb_INS_Executable_Code_CRC, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 1, mcb_bit_status.mcb_INS_Configuration_File_CRC, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 2, mcb_bit_status.mcb_Coefficient, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 4, mcb_bit_status.mcb_IMU_Configuration_Table_CRC, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 5, mcb_bit_status.mcb_IMU_Normal_Mode_Code_CRC, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 6, mcb_bit_status.mcb_External_IMU_GFR_Sync, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 7, mcb_bit_status.mcb_External_IMU_CFR_Sync, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 8, mcb_bit_status.mcb_IMU_FPGA, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 10, mcb_bit_status.mcb_IMU_Serial_Communications, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 11, mcb_bit_status.mcb_IMU_SPI, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 12, mcb_bit_status.mcb_INS_Memory_Interface, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 13, mcb_bit_status.mcb_INS_Memory_Static_Data, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 15, mcb_bit_status.mcb_IMU_Non_Comprehensive_RAM, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 16, mcb_bit_status.mcb_INS_Temperature, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 17, mcb_bit_status.mcb_IMU_Loop_Completion, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 18, mcb_bit_status.mcb_IMU_WDT, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 19, mcb_bit_status.mcb_IMU_Processor, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 20, mcb_bit_status.mcb_INS_Processor, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 21, mcb_bit_status.mcb_Processor_Interrupt, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 22, mcb_bit_status.mcb_Frame_Timer, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 23, mcb_bit_status.mcb_Time_Tagging, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 24, mcb_bit_status.mcb_IMU_Stack_Overflow, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 25, mcb_bit_status.mcb_Power_Up_BIT_Timer, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 26, mcb_bit_status.mcb_INS_Stack_Overflow, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 27, mcb_bit_status.mcb_Programmable_Logic_Read_Back, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 28, mcb_bit_status.mcb_INS_Loop_Completion, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 32, mcb_bit_status.mcb_First_Stage_Boot_Loader, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 33, mcb_bit_status.mcb_Flash_Loader_Table, status_ok);
	mcb_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(mcb_bit_status_tmp, 34, mcb_bit_status.mcb_Register_Initialization_Table, status_ok);
	bb.setOffset(152);	bb.put(mcb_bit_status_tmp);


	uint32_t gps_bit_status_tmp[4] = { 0 }; // buffer holding the custom bitfield
	gps_bit_status_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(gps_bit_status_tmp[0], 0, gps_bit_status.gpssw_GPS_Function, status_ok);
	gps_bit_status_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(gps_bit_status_tmp[0], 1, gps_bit_status.gpssw_GPS_Communication, status_ok);
	gps_bit_status_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(gps_bit_status_tmp[0], 2, gps_bit_status.gpssw_GPS_Time_Mark, status_ok);
	gps_bit_status_tmp[0] = ECTOS::BIT_UTILITIES::PackBool(gps_bit_status_tmp[0], 3, gps_bit_status.gpssw_GPS_T20_Synchronization, status_ok);
	bb.setOffset(160);	bb.put(gps_bit_status_tmp);


	uint64_t gyro_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 0, gyro_bit_status.gyro_status_Gyro_Run_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 1, gyro_bit_status.gyro_status_Gyro_Run_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 2, gyro_bit_status.gyro_status_Gyro_Run_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 3, gyro_bit_status.gyro_status_Gyro_Register_Read_Back_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 4, gyro_bit_status.gyro_status_Gyro_Register_Read_Back_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 5, gyro_bit_status.gyro_status_Gyro_Register_Read_Back_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 6, gyro_bit_status.gyro_status_Gyro_Temperature_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 7, gyro_bit_status.gyro_status_Gyro_Temperature_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 8, gyro_bit_status.gyro_status_Gyro_Temperature_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 9, gyro_bit_status.gyro_status_Gyro_AGC_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 10, gyro_bit_status.gyro_status_Gyro_AGC_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 11, gyro_bit_status.gyro_status_Gyro_AGC_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 12, gyro_bit_status.gyro_status_Gyro_Motor_Bias_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 13, gyro_bit_status.gyro_status_Gyro_Motor_Bias_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 14, gyro_bit_status.gyro_status_Gyro_Motor_Bias_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 15, gyro_bit_status.gyro_status_Gyro_NVREF_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 16, gyro_bit_status.gyro_status_Gyro_NVREF_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 17, gyro_bit_status.gyro_status_Gyro_NVREF_Z, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 21, gyro_bit_status.gyro_status_Gyro_Sensor_X, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 22, gyro_bit_status.gyro_status_Gyro_Sensor_Y, status_ok);
	gyro_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(gyro_bit_status_tmp, 23, gyro_bit_status.gyro_status_Gyro_Sensor_Z, status_ok);
	bb.setOffset(176);	bb.put(gyro_bit_status_tmp);


	uint64_t accelerometer_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 0, accelerometer_bit_status.accel_status_Accel_Register_Read_Back_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 1, accelerometer_bit_status.accel_status_Accel_Register_Read_Back_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 2, accelerometer_bit_status.accel_status_Accel_Register_Read_Back_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 3, accelerometer_bit_status.accel_status_Accel_Temperature_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 4, accelerometer_bit_status.accel_status_Accel_Temperature_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 5, accelerometer_bit_status.accel_status_Accel_Temperature_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 6, accelerometer_bit_status.accel_status_Accel_Sensor_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 7, accelerometer_bit_status.accel_status_Accel_Sensor_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 8, accelerometer_bit_status.accel_status_Accel_Sensor_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 9, accelerometer_bit_status.accel_status_Accel_AC_Stim_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 10, accelerometer_bit_status.accel_status_Accel_AC_Stim_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 11, accelerometer_bit_status.accel_status_Accel_AC_Stim_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 15, accelerometer_bit_status.accel_status_Accel_Analog_Continuity_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 16, accelerometer_bit_status.accel_status_Accel_Analog_Continuity_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 17, accelerometer_bit_status.accel_status_Accel_Analog_Continuity_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 18, accelerometer_bit_status.accel_status_Accel_Bypass_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 19, accelerometer_bit_status.accel_status_Accel_Bypass_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 20, accelerometer_bit_status.accel_status_Accel_Bypass_Z, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 21, accelerometer_bit_status.accel_status_Accel_Level_X, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 22, accelerometer_bit_status.accel_status_Accel_Level_Y, status_ok);
	accelerometer_bit_status_tmp = ECTOS::BIT_UTILITIES::PackBool(accelerometer_bit_status_tmp, 23, accelerometer_bit_status.accel_status_Accel_Level_Z, status_ok);
	bb.setOffset(184);	bb.put(accelerometer_bit_status_tmp);

	bb.setOffset(192);	bb.put(accel_x_temperature_degC);
	bb.setOffset(196);	bb.put(accel_y_temperature_degC);
	bb.setOffset(200);	bb.put(accel_z_temperature_degC);
	bb.setOffset(204);	bb.put(gyro_x_temperature_degC);
	bb.setOffset(208);	bb.put(gyro_y_temperature_degC);
	bb.setOffset(212);	bb.put(gyro_z_temperature_degC);
	bb.setOffset(216);	bb.put(MDTC_zero_vel_meas_noiseX);
	bb.setOffset(217);	bb.put(MDTC_inv_nominal_signal_filter_fn);
	bb.setOffset(218);	bb.put(MDTC_instant_signal_filter_fn);
	bb.setOffset(219);	bb.put(MDTC_settling_time);
	Checksum = computeChecksum((uint32_t*)buffer, 55);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_2011::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 220) return -2;

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
	bb.setOffset(16);	bb.get(inertial_data_source);
	bb.setOffset(21);	bb.get(navigation_initialization_status);
	bb.setOffset(22);	bb.get(navErrorModel);
	bb.setOffset(23);	bb.get(INSMode);

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
	InsGnssSummary.SnapbackStatus = ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 17, 19,  status_ok);
	InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(ECTOS::BIT_UTILITIES::UnPack(InsGnssSummary_tmp, 28, 31,  status_ok));
	bb.setOffset(32);	bb.get(systemTov);
	bb.setOffset(40);	bb.get(gpsTov);
	bb.setOffset(48);	bb.get(gps_week);
	bb.setOffset(52);	bb.get(power_cycle_count);
	bb.setOffset(56);	bb.get(eti);
	bb.setOffset(64);	bb.get(ins_device_temperature);
	bb.setOffset(68);	bb.get(bitMode);
	bb.setOffset(72);	bb.get(ins_blended_fom);
	bb.setOffset(76);	bb.get(gps_fom);
	bb.setOffset(80);	bb.get(utc_tfom);
	bb.setOffset(88);	bb.get(MDT_stationary_instantaneous_signal);
	bb.setOffset(89);	bb.get(MDT_stationary_accel_mag);
	bb.setOffset(90);	bb.get(MDT_stationary_speed_stdv);
	bb.setOffset(91);	bb.get(MDT_stationary_rotation_rate);

	uint32_t MessageWord1_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(92);	bb.get(MessageWord1_tmp);
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
	bb.setOffset(96);	bb.get(MessageWord2_tmp);
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

	uint32_t imu_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(100);	bb.get(imu_bit_status_tmp);
	imu_bit_status.imu_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 0,  status_ok);
	imu_bit_status.processor_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 1,  status_ok);
	imu_bit_status.memory_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 2,  status_ok);
	imu_bit_status.other_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 3,  status_ok);
	imu_bit_status.accelerometer_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 4,  status_ok);
	imu_bit_status.gyro_failed = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 5,  status_ok);
	imu_bit_status.external_sync_fault = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 6,  status_ok);
	imu_bit_status.imu_bit_mode = ECTOS::BIT_UTILITIES::UnPackBool(imu_bit_status_tmp, 28,  status_ok);

	uint32_t aiding_sources_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(104);	bb.get(aiding_sources_tmp);
	aiding_sources.zero_velocity = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 0,  status_ok);
	aiding_sources.GNSS_PVT_P = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 1,  status_ok);
	aiding_sources.GNSS_PVT_V = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 2,  status_ok);
	aiding_sources.GNSS_ATT_A = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 3,  status_ok);
	aiding_sources.GNSS_pseudo_range = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 4,  status_ok);
	aiding_sources.GNSS_delta_range = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 5,  status_ok);
	aiding_sources.baro_altitude = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 7,  status_ok);
	aiding_sources.position_match = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 8,  status_ok);
	aiding_sources.velocity_match = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 9,  status_ok);
	aiding_sources.attitude_match = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 10,  status_ok);
	aiding_sources.mag_heading = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 11,  status_ok);
	aiding_sources.static_heading = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 12,  status_ok);
	aiding_sources.motion_detect = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 13,  status_ok);
	aiding_sources.odometer = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 15,  status_ok);
	aiding_sources.land_constraints = ECTOS::BIT_UTILITIES::UnPackBool(aiding_sources_tmp, 16,  status_ok);
	bb.setOffset(112);	bb.get(number_of_satellites_in_blended_sol);

	uint32_t pr_validity_by_gps_receiver_channel_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(116);	bb.get(pr_validity_by_gps_receiver_channel_tmp);
	pr_validity_by_gps_receiver_channel.channel_0 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 0,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_1 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 1,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_2 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 2,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_3 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 3,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_4 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 4,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_5 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 5,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_6 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 6,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_7 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 7,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_8 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 8,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_9 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 9,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_10 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 10,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_11 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 11,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_12 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 12,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_13 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 13,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_14 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 14,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_15 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 15,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_16 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 16,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_17 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 17,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_18 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 18,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_19 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 19,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_20 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 20,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_21 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 21,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_22 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 22,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_23 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 23,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_24 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 24,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_25 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 25,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_26 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 26,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_27 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 27,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_28 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 28,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_29 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 29,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_30 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 30,  status_ok);
	pr_validity_by_gps_receiver_channel.channel_31 = ECTOS::BIT_UTILITIES::UnPackBool(pr_validity_by_gps_receiver_channel_tmp, 31,  status_ok);

	uint32_t dr_validity_by_gps_receiver_channel_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(120);	bb.get(dr_validity_by_gps_receiver_channel_tmp);
	dr_validity_by_gps_receiver_channel.channel_0 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 0,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_1 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 1,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_2 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 2,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_3 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 3,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_4 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 4,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_5 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 5,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_6 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 6,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_7 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 7,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_8 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 8,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_9 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 9,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_10 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 10,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_11 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 11,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_12 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 12,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_13 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 13,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_14 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 14,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_15 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 15,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_16 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 16,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_17 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 17,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_18 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 18,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_19 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 19,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_20 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 20,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_21 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 21,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_22 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 22,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_23 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 23,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_24 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 24,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_25 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 25,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_26 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 26,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_27 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 27,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_28 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 28,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_29 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 29,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_30 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 30,  status_ok);
	dr_validity_by_gps_receiver_channel.channel_31 = ECTOS::BIT_UTILITIES::UnPackBool(dr_validity_by_gps_receiver_channel_tmp, 31,  status_ok);

	uint32_t ssbl_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(124);	bb.get(ssbl_status_tmp);
	ssbl_status.flash_register_init_table_missing = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 0,  status_ok);
	ssbl_status.flash_register_init_file_checksum_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 1,  status_ok);
	ssbl_status.flash_table_engry_checksum_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 2,  status_ok);
	ssbl_status.flash_table_entry_address_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 3,  status_ok);
	ssbl_status.ssbl_register_init_table_missing = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 4,  status_ok);
	ssbl_status.ssbl_register_init_file_checksum_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 5,  status_ok);
	ssbl_status.ssbl_table_engry_checksum_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 6,  status_ok);
	ssbl_status.ssbl_table_entry_address_error = ECTOS::BIT_UTILITIES::UnPackBool(ssbl_status_tmp, 7,  status_ok);
	bb.setOffset(128);	bb.get(solution_convergence);
	bb.setOffset(132);	bb.get(attitude_fom);

	uint32_t gnssr_aiding_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(136);	bb.get(gnssr_aiding_status_tmp);
	gnssr_aiding_status.gnssr_receiver_inertial_aiding_capable = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 0,  status_ok);
	gnssr_aiding_status.gnssr_inertial_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 1,  status_ok);
	gnssr_aiding_status.gnssr_inertial_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 2,  status_ok);
	gnssr_aiding_status.gnssr_drs_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 3,  status_ok);
	gnssr_aiding_status.gnssr_drs_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 4,  status_ok);
	gnssr_aiding_status.gnssr_baro_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 5,  status_ok);
	gnssr_aiding_status.gnssr_baro_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 6,  status_ok);
	gnssr_aiding_status.gnssr_tas_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 7,  status_ok);
	gnssr_aiding_status.gnssr_tas_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 8,  status_ok);
	gnssr_aiding_status.gnssr_mag_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 9,  status_ok);
	gnssr_aiding_status.gnssr_mag_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 10,  status_ok);
	gnssr_aiding_status.gnssr_attitude_aiding_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 11,  status_ok);
	gnssr_aiding_status.gnssr_attitude_aiding_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 12,  status_ok);
	gnssr_aiding_status.gnssr_std_frequency_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 13,  status_ok);
	gnssr_aiding_status.gnssr_std_frequency_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 14,  status_ok);
	gnssr_aiding_status.gnssr_std_time_valid = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 15,  status_ok);
	gnssr_aiding_status.gnssr_std_time_in_use = ECTOS::BIT_UTILITIES::UnPackBool(gnssr_aiding_status_tmp, 16,  status_ok);

	uint64_t fsbl_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(140);	bb.get(fsbl_status_tmp);
	fsbl_status.fsbl_boot_rom_error_code = ECTOS::BIT_UTILITIES::UnPack(fsbl_status_tmp, 0, 15,  status_ok);
	fsbl_status.fslb_system_wdt_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 16,  status_ok);
	fsbl_status.fsbl_apu_0_wdt_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 17,  status_ok);
	fsbl_status.fsbl_apu_1_wdt_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 18,  status_ok);
	fsbl_status.fsbl_software_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 19,  status_ok);
	fsbl_status.fsbl_debug_system_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 20,  status_ok);
	fsbl_status.fsbl_external_system_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 21,  status_ok);
	fsbl_status.fsbl_power_on_reset = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 22,  status_ok);
	fsbl_status.fsbl_reboot_state = ECTOS::BIT_UTILITIES::UnPack(fsbl_status_tmp, 24, 31,  status_ok);
	fsbl_status.fsbl_pmic_boost_status = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 32,  status_ok);
	fsbl_status.fsbl_gpga_initialization_done = ECTOS::BIT_UTILITIES::UnPackBool(fsbl_status_tmp, 33,  status_ok);

	uint32_t system_bit_summary_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(148);	bb.get(system_bit_summary_tmp);
	system_bit_summary.Go_NoGo_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 0,  status_ok);
	system_bit_summary.MPB_Subsystem_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 1,  status_ok);
	system_bit_summary.GPS_Subsystem_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 2,  status_ok);
	system_bit_summary.Gyro_Subsystem_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 3,  status_ok);
	system_bit_summary.Accelerometer_Subsystem_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 4,  status_ok);
	system_bit_summary.IMU_Subsystem_BIT_Summary = ECTOS::BIT_UTILITIES::UnPackBool(system_bit_summary_tmp, 6,  status_ok);

	uint64_t mcb_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(152);	bb.get(mcb_bit_status_tmp);
	mcb_bit_status.mcb_INS_Executable_Code_CRC = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 0,  status_ok);
	mcb_bit_status.mcb_INS_Configuration_File_CRC = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 1,  status_ok);
	mcb_bit_status.mcb_Coefficient = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 2,  status_ok);
	mcb_bit_status.mcb_IMU_Configuration_Table_CRC = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 4,  status_ok);
	mcb_bit_status.mcb_IMU_Normal_Mode_Code_CRC = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 5,  status_ok);
	mcb_bit_status.mcb_External_IMU_GFR_Sync = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 6,  status_ok);
	mcb_bit_status.mcb_External_IMU_CFR_Sync = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 7,  status_ok);
	mcb_bit_status.mcb_IMU_FPGA = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 8,  status_ok);
	mcb_bit_status.mcb_IMU_Serial_Communications = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 10,  status_ok);
	mcb_bit_status.mcb_IMU_SPI = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 11,  status_ok);
	mcb_bit_status.mcb_INS_Memory_Interface = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 12,  status_ok);
	mcb_bit_status.mcb_INS_Memory_Static_Data = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 13,  status_ok);
	mcb_bit_status.mcb_IMU_Non_Comprehensive_RAM = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 15,  status_ok);
	mcb_bit_status.mcb_INS_Temperature = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 16,  status_ok);
	mcb_bit_status.mcb_IMU_Loop_Completion = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 17,  status_ok);
	mcb_bit_status.mcb_IMU_WDT = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 18,  status_ok);
	mcb_bit_status.mcb_IMU_Processor = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 19,  status_ok);
	mcb_bit_status.mcb_INS_Processor = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 20,  status_ok);
	mcb_bit_status.mcb_Processor_Interrupt = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 21,  status_ok);
	mcb_bit_status.mcb_Frame_Timer = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 22,  status_ok);
	mcb_bit_status.mcb_Time_Tagging = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 23,  status_ok);
	mcb_bit_status.mcb_IMU_Stack_Overflow = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 24,  status_ok);
	mcb_bit_status.mcb_Power_Up_BIT_Timer = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 25,  status_ok);
	mcb_bit_status.mcb_INS_Stack_Overflow = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 26,  status_ok);
	mcb_bit_status.mcb_Programmable_Logic_Read_Back = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 27,  status_ok);
	mcb_bit_status.mcb_INS_Loop_Completion = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 28,  status_ok);
	mcb_bit_status.mcb_First_Stage_Boot_Loader = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 32,  status_ok);
	mcb_bit_status.mcb_Flash_Loader_Table = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 33,  status_ok);
	mcb_bit_status.mcb_Register_Initialization_Table = ECTOS::BIT_UTILITIES::UnPackBool(mcb_bit_status_tmp, 34,  status_ok);

	uint32_t gps_bit_status_tmp[4] = { 0 }; // buffer holding the custom bitfield
	bb.setOffset(160);	bb.get(gps_bit_status_tmp);
	gps_bit_status.gpssw_GPS_Function = ECTOS::BIT_UTILITIES::UnPackBool(gps_bit_status_tmp[0], 0,  status_ok);
	gps_bit_status.gpssw_GPS_Communication = ECTOS::BIT_UTILITIES::UnPackBool(gps_bit_status_tmp[0], 1,  status_ok);
	gps_bit_status.gpssw_GPS_Time_Mark = ECTOS::BIT_UTILITIES::UnPackBool(gps_bit_status_tmp[0], 2,  status_ok);
	gps_bit_status.gpssw_GPS_T20_Synchronization = ECTOS::BIT_UTILITIES::UnPackBool(gps_bit_status_tmp[0], 3,  status_ok);

	uint64_t gyro_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(176);	bb.get(gyro_bit_status_tmp);
	gyro_bit_status.gyro_status_Gyro_Run_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 0,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Run_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 1,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Run_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 2,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Register_Read_Back_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 3,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Register_Read_Back_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 4,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Register_Read_Back_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 5,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Temperature_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 6,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Temperature_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 7,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Temperature_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 8,  status_ok);
	gyro_bit_status.gyro_status_Gyro_AGC_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 9,  status_ok);
	gyro_bit_status.gyro_status_Gyro_AGC_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 10,  status_ok);
	gyro_bit_status.gyro_status_Gyro_AGC_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 11,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Motor_Bias_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 12,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Motor_Bias_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 13,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Motor_Bias_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 14,  status_ok);
	gyro_bit_status.gyro_status_Gyro_NVREF_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 15,  status_ok);
	gyro_bit_status.gyro_status_Gyro_NVREF_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 16,  status_ok);
	gyro_bit_status.gyro_status_Gyro_NVREF_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 17,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Sensor_X = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 21,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Sensor_Y = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 22,  status_ok);
	gyro_bit_status.gyro_status_Gyro_Sensor_Z = ECTOS::BIT_UTILITIES::UnPackBool(gyro_bit_status_tmp, 23,  status_ok);

	uint64_t accelerometer_bit_status_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(184);	bb.get(accelerometer_bit_status_tmp);
	accelerometer_bit_status.accel_status_Accel_Register_Read_Back_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 0,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Register_Read_Back_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 1,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Register_Read_Back_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 2,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Temperature_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 3,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Temperature_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 4,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Temperature_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 5,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Sensor_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 6,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Sensor_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 7,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Sensor_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 8,  status_ok);
	accelerometer_bit_status.accel_status_Accel_AC_Stim_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 9,  status_ok);
	accelerometer_bit_status.accel_status_Accel_AC_Stim_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 10,  status_ok);
	accelerometer_bit_status.accel_status_Accel_AC_Stim_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 11,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Analog_Continuity_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 15,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Analog_Continuity_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 16,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Analog_Continuity_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 17,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Bypass_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 18,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Bypass_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 19,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Bypass_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 20,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Level_X = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 21,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Level_Y = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 22,  status_ok);
	accelerometer_bit_status.accel_status_Accel_Level_Z = ECTOS::BIT_UTILITIES::UnPackBool(accelerometer_bit_status_tmp, 23,  status_ok);
	bb.setOffset(192);	bb.get(accel_x_temperature_degC);
	bb.setOffset(196);	bb.get(accel_y_temperature_degC);
	bb.setOffset(200);	bb.get(accel_z_temperature_degC);
	bb.setOffset(204);	bb.get(gyro_x_temperature_degC);
	bb.setOffset(208);	bb.get(gyro_y_temperature_degC);
	bb.setOffset(212);	bb.get(gyro_z_temperature_degC);
	bb.setOffset(216);	bb.get(MDTC_zero_vel_meas_noiseX);
	bb.setOffset(217);	bb.get(MDTC_inv_nominal_signal_filter_fn);
	bb.setOffset(218);	bb.get(MDTC_instant_signal_filter_fn);
	bb.setOffset(219);	bb.get(MDTC_settling_time);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 55);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

