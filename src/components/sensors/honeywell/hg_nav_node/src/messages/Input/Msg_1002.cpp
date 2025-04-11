#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1002.h>
#include <hg_nav_node/gps_mode_control_t.h>
#include <hg_nav_node/ins_mode_control_t.h>
#include <hg_nav_node/measurement_noise_input_indicator_t.h>
#include <hg_nav_node/navigation_aiding_sources_t.h>
hg_nav_node::Msg_1002 msgStruct_1002;

ros::Subscriber Msg_1002_sub;
void init_1002(ros::NodeHandle * n){
	Msg_1002_sub = n->subscribe(MSG_1002_PATH, 5, Msg_1002_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1002_PATH);
	return;
}

void stop_1002(void){
	Msg_1002_sub.shutdown();
	ROS_INFO("0x1002 stopped");
	return;
}

// Msg_1002 to Topic
void convert(Msg_1002 messageIn, hg_nav_node::Msg_1002 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INS_Mode.value = static_cast<uint8_t>(messageIn.INS_Mode);
	messageOut->gps_mode_control.value = static_cast<uint8_t>(messageIn.gps_mode_control);
	messageOut->reset_ins_gps = messageIn.reset_ins_gps;
	messageOut->clear_bit_history = messageIn.clear_bit_history;
	messageOut->time_mark_pps_select = messageIn.time_mark_pps_select;
	messageOut->gps_inertial_aiding = messageIn.gps_inertial_aiding;
	messageOut->coarse_level_duration = messageIn.coarse_level_duration;

	messageOut->navigation_aiding_sources.Enable_GNSS_Psuedorange = messageIn.navigation_aiding_sources.Enable_GNSS_Psuedorange;
	messageOut->navigation_aiding_sources.Enable_GNSS_Deltarange = messageIn.navigation_aiding_sources.Enable_GNSS_Deltarange;
	messageOut->navigation_aiding_sources.Enable_GNSS_Code_Delay = messageIn.navigation_aiding_sources.Enable_GNSS_Code_Delay;
	messageOut->navigation_aiding_sources.Enable_GNSS_Carrier_Frequency = messageIn.navigation_aiding_sources.Enable_GNSS_Carrier_Frequency;
	messageOut->navigation_aiding_sources.Enable_GNSS_Velocity = messageIn.navigation_aiding_sources.Enable_GNSS_Velocity;
	messageOut->navigation_aiding_sources.Enable_GNSS_Position = messageIn.navigation_aiding_sources.Enable_GNSS_Position;
	messageOut->navigation_aiding_sources.Enable_GNSS_Attitude = messageIn.navigation_aiding_sources.Enable_GNSS_Attitude;
	messageOut->navigation_aiding_sources.Enable_Attitude_Reference = messageIn.navigation_aiding_sources.Enable_Attitude_Reference;
	messageOut->navigation_aiding_sources.Enable_Velocity_Reference = messageIn.navigation_aiding_sources.Enable_Velocity_Reference;
	messageOut->navigation_aiding_sources.Enable_Position_Reference = messageIn.navigation_aiding_sources.Enable_Position_Reference;
	messageOut->navigation_aiding_sources.Enable_ZUPT = messageIn.navigation_aiding_sources.Enable_ZUPT;
	messageOut->navigation_aiding_sources.Enable_Zero_Heading_Change = messageIn.navigation_aiding_sources.Enable_Zero_Heading_Change;
	messageOut->navigation_aiding_sources.Enable_Barometric_Altitude = messageIn.navigation_aiding_sources.Enable_Barometric_Altitude;
	messageOut->navigation_aiding_sources.Enable_Odometer = messageIn.navigation_aiding_sources.Enable_Odometer;
	messageOut->navigation_aiding_sources.Enable_Magnetic_Heading = messageIn.navigation_aiding_sources.Enable_Magnetic_Heading;
	messageOut->navigation_aiding_sources.Enable_Automotive_Land_Car_Profile = messageIn.navigation_aiding_sources.Enable_Automotive_Land_Car_Profile;
	messageOut->navigation_aiding_sources.Enable_Motion_Detect = messageIn.navigation_aiding_sources.Enable_Motion_Detect;
	messageOut->navigation_aiding_sources.Enable_Aiding_Source = messageIn.navigation_aiding_sources.Enable_Aiding_Source;
	messageOut->dyn_track_loop_start_time = messageIn.dyn_track_loop_start_time;
	messageOut->dyn_track_loop_control_ev_duration = messageIn.dyn_track_loop_control_ev_duration;
	messageOut->inertial_shock_event_start_time = messageIn.inertial_shock_event_start_time;
	messageOut->inertial_shock_event_duration = messageIn.inertial_shock_event_duration;
	messageOut->inertial_shock_event_int_velocity_unc = messageIn.inertial_shock_event_int_velocity_unc;
	messageOut->inertial_shock_event_int_attitude_unc = messageIn.inertial_shock_event_int_attitude_unc;
	messageOut->inertial_vib_ev_start_time = messageIn.inertial_vib_ev_start_time;
	messageOut->inertial_vib_event_gyro_bias_unc = messageIn.inertial_vib_event_gyro_bias_unc;
	messageOut->inertial_vib_event_accel_bias_unc = messageIn.inertial_vib_event_accel_bias_unc;
	messageOut->store_input_motion_detect_zupting_to_flash = messageIn.store_input_motion_detect_zupting_to_flash;
	messageOut->motion_detect_profile = messageIn.motion_detect_profile;

	messageOut->measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT;
	messageOut->measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT;
	messageOut->measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH;
	messageOut->measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD = messageIn.measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD;
	messageOut->measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH;
	messageOut->measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH;
	messageOut->measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT;
	messageOut->zero_velocity_measurement_noise_stdv = messageIn.zero_velocity_measurement_noise_stdv;
	messageOut->zero_heading_change_measurement_noise_stdv = messageIn.zero_heading_change_measurement_noise_stdv;
	messageOut->motion_detect_settling_time = messageIn.motion_detect_settling_time;
	messageOut->motion_detect_dtheta_threshold = messageIn.motion_detect_dtheta_threshold;
	messageOut->motion_detect_nominal_w_fn = messageIn.motion_detect_nominal_w_fn;
	messageOut->motion_detect_instant_w_fn = messageIn.motion_detect_instant_w_fn;
	messageOut->motion_detect_instant_threshold = messageIn.motion_detect_instant_threshold;
	messageOut->motion_detect_accel_threshold = messageIn.motion_detect_accel_threshold;
	messageOut->motion_detect_speed_threshold = messageIn.motion_detect_speed_threshold;
	messageOut->motion_detect_odo_dist_threshold = messageIn.motion_detect_odo_dist_threshold;
}

// Topic to Msg_1002
void convert(hg_nav_node::Msg_1002 messageIn, Msg_1002 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INS_Mode = static_cast<ins_mode_control_t>(messageIn.INS_Mode.value);
	messageOut->gps_mode_control = static_cast<gps_mode_control_t>(messageIn.gps_mode_control.value);
	messageOut->reset_ins_gps = messageIn.reset_ins_gps;
	messageOut->clear_bit_history = messageIn.clear_bit_history;
	messageOut->time_mark_pps_select = messageIn.time_mark_pps_select;
	messageOut->gps_inertial_aiding = messageIn.gps_inertial_aiding;
	messageOut->coarse_level_duration = messageIn.coarse_level_duration;

	messageOut->navigation_aiding_sources.Enable_GNSS_Psuedorange = messageIn.navigation_aiding_sources.Enable_GNSS_Psuedorange;
	messageOut->navigation_aiding_sources.Enable_GNSS_Deltarange = messageIn.navigation_aiding_sources.Enable_GNSS_Deltarange;
	messageOut->navigation_aiding_sources.Enable_GNSS_Code_Delay = messageIn.navigation_aiding_sources.Enable_GNSS_Code_Delay;
	messageOut->navigation_aiding_sources.Enable_GNSS_Carrier_Frequency = messageIn.navigation_aiding_sources.Enable_GNSS_Carrier_Frequency;
	messageOut->navigation_aiding_sources.Enable_GNSS_Velocity = messageIn.navigation_aiding_sources.Enable_GNSS_Velocity;
	messageOut->navigation_aiding_sources.Enable_GNSS_Position = messageIn.navigation_aiding_sources.Enable_GNSS_Position;
	messageOut->navigation_aiding_sources.Enable_GNSS_Attitude = messageIn.navigation_aiding_sources.Enable_GNSS_Attitude;
	messageOut->navigation_aiding_sources.Enable_Attitude_Reference = messageIn.navigation_aiding_sources.Enable_Attitude_Reference;
	messageOut->navigation_aiding_sources.Enable_Velocity_Reference = messageIn.navigation_aiding_sources.Enable_Velocity_Reference;
	messageOut->navigation_aiding_sources.Enable_Position_Reference = messageIn.navigation_aiding_sources.Enable_Position_Reference;
	messageOut->navigation_aiding_sources.Enable_ZUPT = messageIn.navigation_aiding_sources.Enable_ZUPT;
	messageOut->navigation_aiding_sources.Enable_Zero_Heading_Change = messageIn.navigation_aiding_sources.Enable_Zero_Heading_Change;
	messageOut->navigation_aiding_sources.Enable_Barometric_Altitude = messageIn.navigation_aiding_sources.Enable_Barometric_Altitude;
	messageOut->navigation_aiding_sources.Enable_Odometer = messageIn.navigation_aiding_sources.Enable_Odometer;
	messageOut->navigation_aiding_sources.Enable_Magnetic_Heading = messageIn.navigation_aiding_sources.Enable_Magnetic_Heading;
	messageOut->navigation_aiding_sources.Enable_Automotive_Land_Car_Profile = messageIn.navigation_aiding_sources.Enable_Automotive_Land_Car_Profile;
	messageOut->navigation_aiding_sources.Enable_Motion_Detect = messageIn.navigation_aiding_sources.Enable_Motion_Detect;
	messageOut->navigation_aiding_sources.Enable_Aiding_Source = messageIn.navigation_aiding_sources.Enable_Aiding_Source;
	messageOut->dyn_track_loop_start_time = messageIn.dyn_track_loop_start_time;
	messageOut->dyn_track_loop_control_ev_duration = messageIn.dyn_track_loop_control_ev_duration;
	messageOut->inertial_shock_event_start_time = messageIn.inertial_shock_event_start_time;
	messageOut->inertial_shock_event_duration = messageIn.inertial_shock_event_duration;
	messageOut->inertial_shock_event_int_velocity_unc = messageIn.inertial_shock_event_int_velocity_unc;
	messageOut->inertial_shock_event_int_attitude_unc = messageIn.inertial_shock_event_int_attitude_unc;
	messageOut->inertial_vib_ev_start_time = messageIn.inertial_vib_ev_start_time;
	messageOut->inertial_vib_event_gyro_bias_unc = messageIn.inertial_vib_event_gyro_bias_unc;
	messageOut->inertial_vib_event_accel_bias_unc = messageIn.inertial_vib_event_accel_bias_unc;
	messageOut->store_input_motion_detect_zupting_to_flash = messageIn.store_input_motion_detect_zupting_to_flash;
	messageOut->motion_detect_profile = messageIn.motion_detect_profile;

	messageOut->measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_ZERO_VELOCITY_NOISE_INPUT;
	messageOut->measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT;
	messageOut->measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_MEAS_NOISE_TO_FLASH;
	messageOut->measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD = messageIn.measurement_noise_input_indicator.ENABLE_MOTION_DETECT_THRESHOLD;
	messageOut->measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH;
	messageOut->measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH = messageIn.measurement_noise_input_indicator.SAVE_AIDING_SOURCES_TO_FLASH;
	messageOut->measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT = messageIn.measurement_noise_input_indicator.ENABLE_DISABLE_NOISE_SET_TO_INPUT;
	messageOut->zero_velocity_measurement_noise_stdv = messageIn.zero_velocity_measurement_noise_stdv;
	messageOut->zero_heading_change_measurement_noise_stdv = messageIn.zero_heading_change_measurement_noise_stdv;
	messageOut->motion_detect_settling_time = messageIn.motion_detect_settling_time;
	messageOut->motion_detect_dtheta_threshold = messageIn.motion_detect_dtheta_threshold;
	messageOut->motion_detect_nominal_w_fn = messageIn.motion_detect_nominal_w_fn;
	messageOut->motion_detect_instant_w_fn = messageIn.motion_detect_instant_w_fn;
	messageOut->motion_detect_instant_threshold = messageIn.motion_detect_instant_threshold;
	messageOut->motion_detect_accel_threshold = messageIn.motion_detect_accel_threshold;
	messageOut->motion_detect_speed_threshold = messageIn.motion_detect_speed_threshold;
	messageOut->motion_detect_odo_dist_threshold = messageIn.motion_detect_odo_dist_threshold;
}

void Msg_1002_sub_callback(const hg_nav_node::Msg_1002::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1002 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1002 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1002 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
