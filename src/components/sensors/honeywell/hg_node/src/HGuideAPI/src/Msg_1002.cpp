#include <include/HGuideAPI.h>
#include <include/Msg_1002.h>

#include <cmath>
#include <cstring>

#include <include/utils/bit_utilities.h>
#include <include/utils/byte_buffer.h>
#include <include/utils/ectos_constants.h>
#include <include/utils/Checksum.h>


// Definition for Static constants
const uint32_t Msg_1002::AddressId;
const uint32_t Msg_1002::MessageId;
const uint32_t Msg_1002::MessageLength;

Msg_1002::Msg_1002()
{
	Default();
}

void Msg_1002::Default()
{
	Checksum = 0;
	INS_Mode = static_cast<ins_mode_control_t>(0);
	gps_mode_control = static_cast<gps_mode_control_t>(0);
	reset_ins_gps = 0;
	clear_bit_history = 0;
	time_mark_pps_select = 0;
	gps_inertial_aiding = 0;
	coarse_level_duration = 2;
	navigation_aiding_sources.Default();
	dyn_track_loop_start_time = 0;
	dyn_track_loop_control_ev_duration = 0;
	store_input_motion_detect_zupting_to_flash = 0;
	motion_detect_profile = 0;
	measurement_noise_input_indicator.Default();
	inertial_shock_event_start_time = -1.0;
	inertial_shock_event_duration = -1.0;
	inertial_shock_event_int_velocity_unc = -1.0;
	inertial_shock_event_int_attitude_unc = -1.0;
	inertial_vib_ev_start_time = -1.0;
	inertial_vib_event_gyro_bias_unc = -1.0;
	inertial_vib_event_accel_bias_unc = -1.0;
	zero_velocity_measurement_noise_stdv = -1.0;
	zero_heading_change_measurement_noise_stdv = 0.0001;
	motion_detect_settling_time = 3.0;
	motion_detect_dtheta_threshold = 0.002;
	motion_detect_nominal_w_fn = 0.2;
	motion_detect_instant_w_fn = 15.0;
	motion_detect_instant_threshold = 0.005;
	motion_detect_accel_threshold = 0.03;
	motion_detect_speed_threshold = 3.0;
	motion_detect_odo_dist_threshold = 0.0;
}

bool Msg_1002::Serialize(unsigned char * buffer, const unsigned int bufferSize)
{
	if (bufferSize < 184) return false;

	std::memset(buffer, 0, bufferSize);

	bool status_ok = true;
	ECTOS::BYTE_BUFFER::ByteOutputBuffer<unsigned int, unsigned char, ECTOS::BYTE_BUFFER::HGuideAPI_endianness> bb(buffer, bufferSize);

	bb.setOffset(0);	bb.put(AddressId);
	bb.setOffset(4);	bb.put(MessageId);
	bb.setOffset(8);	bb.put(MessageLength);
	int cIndex = 12;
	Checksum = 0;
	bb.setOffset(16);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(INS_Mode));
	bb.setOffset(20);	bb.put(static_cast<uint32_t>CHECK_MAX_UINT32(gps_mode_control));
	bb.setOffset(24);	bb.put(reset_ins_gps);
	bb.setOffset(28);	bb.put(clear_bit_history);
	bb.setOffset(36);	bb.put(time_mark_pps_select);
	bb.setOffset(40);	bb.put(gps_inertial_aiding);
	bb.setOffset(44);	bb.put(coarse_level_duration);

	uint32_t navigation_aiding_sources_tmp = 0; // temporary variable holding the custom bitfield
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 1, navigation_aiding_sources.Enable_GNSS_Psuedorange, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 2, navigation_aiding_sources.Enable_GNSS_Deltarange, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 3, navigation_aiding_sources.Enable_GNSS_Code_Delay, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 4, navigation_aiding_sources.Enable_GNSS_Carrier_Frequency, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 5, navigation_aiding_sources.Enable_GNSS_Velocity, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 6, navigation_aiding_sources.Enable_GNSS_Position, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 7, navigation_aiding_sources.Enable_GNSS_Attitude, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 8, navigation_aiding_sources.Enable_Attitude_Reference, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 9, navigation_aiding_sources.Enable_Velocity_Reference, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 10, navigation_aiding_sources.Enable_Position_Reference, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 11, navigation_aiding_sources.Enable_ZUPT, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 12, navigation_aiding_sources.Enable_Zero_Heading_Change, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 13, navigation_aiding_sources.Enable_Barometric_Altitude, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 15, navigation_aiding_sources.Enable_Odometer, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 16, navigation_aiding_sources.Enable_Magnetic_Heading, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 18, navigation_aiding_sources.Enable_Automotive_Land_Car_Profile, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 20, navigation_aiding_sources.Enable_Motion_Detect, status_ok);
	navigation_aiding_sources_tmp = ECTOS::BIT_UTILITIES::PackBool(navigation_aiding_sources_tmp, 31, navigation_aiding_sources.Enable_Aiding_Source, status_ok);
	bb.setOffset(48);	bb.put(navigation_aiding_sources_tmp);

	bb.setOffset(52);	bb.put(dyn_track_loop_start_time);
	bb.setOffset(56);	bb.put(dyn_track_loop_control_ev_duration);
	bb.setOffset(68);	bb.put(inertial_shock_event_start_time);
	bb.setOffset(72);	bb.put(inertial_shock_event_duration);
	bb.setOffset(76);	bb.put(inertial_shock_event_int_velocity_unc);
	bb.setOffset(80);	bb.put(inertial_shock_event_int_attitude_unc);
	bb.setOffset(92);	bb.put(inertial_vib_ev_start_time);
	bb.setOffset(100);	bb.put(inertial_vib_event_gyro_bias_unc);
	bb.setOffset(104);	bb.put(inertial_vib_event_accel_bias_unc);
	bb.setOffset(108);	bb.put(store_input_motion_detect_zupting_to_flash);
	bb.setOffset(112);	bb.put(motion_detect_profile);

	uint32_t measurement_noise_input_indicator_tmp = 0; // temporary variable holding the custom bitfield
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 0, measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 1, measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 2, measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 8, measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 9, measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 15, measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH, status_ok);
	measurement_noise_input_indicator_tmp = ECTOS::BIT_UTILITIES::PackBool(measurement_noise_input_indicator_tmp, 31, measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT, status_ok);
	bb.setOffset(116);	bb.put(measurement_noise_input_indicator_tmp);

	bb.setOffset(120);	bb.put(zero_velocity_measurement_noise_stdv);
	bb.setOffset(124);	bb.put(zero_heading_change_measurement_noise_stdv);
	bb.setOffset(128);	bb.put(motion_detect_settling_time);
	bb.setOffset(132);	bb.put(motion_detect_dtheta_threshold);
	bb.setOffset(136);	bb.put(motion_detect_nominal_w_fn);
	bb.setOffset(140);	bb.put(motion_detect_instant_w_fn);
	bb.setOffset(144);	bb.put(motion_detect_instant_threshold);
	bb.setOffset(148);	bb.put(motion_detect_accel_threshold);
	bb.setOffset(152);	bb.put(motion_detect_speed_threshold);
	bb.setOffset(156);	bb.put(motion_detect_odo_dist_threshold);
	Checksum = computeChecksum((uint32_t*)buffer, 46);
	bb.setOffset(cIndex);		bb.put(Checksum);
	return status_ok;
}

int Msg_1002::Deserialize(unsigned char * buffer, const unsigned int bufferSize)
{
	bool status_ok = true;
	if (bufferSize < 184) return -2;

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
	bb.setOffset(16);	INS_Mode = static_cast<ins_mode_control_t>(bb.get<uint32_t>());
	bb.setOffset(20);	gps_mode_control = static_cast<gps_mode_control_t>(bb.get<uint32_t>());
	bb.setOffset(24);	bb.get(reset_ins_gps);
	bb.setOffset(28);	bb.get(clear_bit_history);
	bb.setOffset(36);	bb.get(time_mark_pps_select);
	bb.setOffset(40);	bb.get(gps_inertial_aiding);
	bb.setOffset(44);	bb.get(coarse_level_duration);

	uint32_t navigation_aiding_sources_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(48);	bb.get(navigation_aiding_sources_tmp);
	navigation_aiding_sources.Enable_GNSS_Psuedorange = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 1,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Deltarange = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 2,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Code_Delay = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 3,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Carrier_Frequency = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 4,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Velocity = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 5,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Position = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 6,  status_ok);
	navigation_aiding_sources.Enable_GNSS_Attitude = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 7,  status_ok);
	navigation_aiding_sources.Enable_Attitude_Reference = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 8,  status_ok);
	navigation_aiding_sources.Enable_Velocity_Reference = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 9,  status_ok);
	navigation_aiding_sources.Enable_Position_Reference = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 10,  status_ok);
	navigation_aiding_sources.Enable_ZUPT = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 11,  status_ok);
	navigation_aiding_sources.Enable_Zero_Heading_Change = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 12,  status_ok);
	navigation_aiding_sources.Enable_Barometric_Altitude = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 13,  status_ok);
	navigation_aiding_sources.Enable_Odometer = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 15,  status_ok);
	navigation_aiding_sources.Enable_Magnetic_Heading = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 16,  status_ok);
	navigation_aiding_sources.Enable_Automotive_Land_Car_Profile = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 18,  status_ok);
	navigation_aiding_sources.Enable_Motion_Detect = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 20,  status_ok);
	navigation_aiding_sources.Enable_Aiding_Source = ECTOS::BIT_UTILITIES::UnPackBool(navigation_aiding_sources_tmp, 31,  status_ok);
	bb.setOffset(52);	bb.get(dyn_track_loop_start_time);
	bb.setOffset(56);	bb.get(dyn_track_loop_control_ev_duration);
	bb.setOffset(68);	bb.get(inertial_shock_event_start_time);
	bb.setOffset(72);	bb.get(inertial_shock_event_duration);
	bb.setOffset(76);	bb.get(inertial_shock_event_int_velocity_unc);
	bb.setOffset(80);	bb.get(inertial_shock_event_int_attitude_unc);
	bb.setOffset(92);	bb.get(inertial_vib_ev_start_time);
	bb.setOffset(100);	bb.get(inertial_vib_event_gyro_bias_unc);
	bb.setOffset(104);	bb.get(inertial_vib_event_accel_bias_unc);
	bb.setOffset(108);	bb.get(store_input_motion_detect_zupting_to_flash);
	bb.setOffset(112);	bb.get(motion_detect_profile);

	uint32_t measurement_noise_input_indicator_tmp = 0; // temporary variable holding the custom bitfield
	bb.setOffset(116);	bb.get(measurement_noise_input_indicator_tmp);
	measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 0,  status_ok);
	measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 1,  status_ok);
	measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 2,  status_ok);
	measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 8,  status_ok);
	measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 9,  status_ok);
	measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 15,  status_ok);
	measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT = ECTOS::BIT_UTILITIES::UnPackBool(measurement_noise_input_indicator_tmp, 31,  status_ok);
	bb.setOffset(120);	bb.get(zero_velocity_measurement_noise_stdv);
	bb.setOffset(124);	bb.get(zero_heading_change_measurement_noise_stdv);
	bb.setOffset(128);	bb.get(motion_detect_settling_time);
	bb.setOffset(132);	bb.get(motion_detect_dtheta_threshold);
	bb.setOffset(136);	bb.get(motion_detect_nominal_w_fn);
	bb.setOffset(140);	bb.get(motion_detect_instant_w_fn);
	bb.setOffset(144);	bb.get(motion_detect_instant_threshold);
	bb.setOffset(148);	bb.get(motion_detect_accel_threshold);
	bb.setOffset(152);	bb.get(motion_detect_speed_threshold);
	bb.setOffset(156);	bb.get(motion_detect_odo_dist_threshold);
	bb.setOffset(12);	bb.get(Checksum);
	uint32_t computedCRC = computeChecksum((uint32_t*)buffer, 46);
	int retVal = 0;
	retVal = constCheck;
	if(computedCRC != Checksum)
		retVal = -1;
	return retVal;
}

