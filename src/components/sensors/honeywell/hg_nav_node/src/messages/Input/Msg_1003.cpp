#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1003.h>
hg_nav_node::Msg_1003 msgStruct_1003;

ros::Subscriber Msg_1003_sub;
void init_1003(ros::NodeHandle * n){
	Msg_1003_sub = n->subscribe(MSG_1003_PATH, 5, Msg_1003_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1003_PATH);
	return;
}

void stop_1003(void){
	Msg_1003_sub.shutdown();
	ROS_INFO("0x1003 stopped");
	return;
}

// Msg_1003 to Topic
void convert(Msg_1003 messageIn, hg_nav_node::Msg_1003 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->anti_jam_selection = messageIn.anti_jam_selection;
	messageOut->gps_antenna_spin = messageIn.gps_antenna_spin;
	messageOut->constellation_type = messageIn.constellation_type;
	messageOut->acquisition_state_selection = messageIn.acquisition_state_selection;
	messageOut->ionosphere_corrections = messageIn.ionosphere_corrections;
	messageOut->post_launch_auto_transition_to_trk = messageIn.post_launch_auto_transition_to_trk;
	messageOut->set_back_shock = messageIn.set_back_shock;
	messageOut->position_uncertainty = messageIn.position_uncertainty;
	messageOut->velocity_uncertainty = messageIn.velocity_uncertainty;
	messageOut->time_uncertainty = messageIn.time_uncertainty;
	messageOut->number_of_acquisition_attempts = messageIn.number_of_acquisition_attempts;
	messageOut->gps_oscillator_calibration_jitter = messageIn.gps_oscillator_calibration_jitter;
	messageOut->gps_oscillator_calibration = messageIn.gps_oscillator_calibration;
	messageOut->gps_code_select = messageIn.gps_code_select;
	messageOut->gps_antenna_frequency_cmd = messageIn.gps_antenna_frequency_cmd;
	messageOut->gps_svid_deselect_1_4 = messageIn.gps_svid_deselect_1_4;
	messageOut->gps_svid_deselect_1_4_2 = messageIn.gps_svid_deselect_1_4_2;
	messageOut->pre_launch_temperature_ramp = messageIn.pre_launch_temperature_ramp;
	messageOut->post_launch_temperature_ramp = messageIn.post_launch_temperature_ramp;
	messageOut->post_launch_power_on_time_delay = messageIn.post_launch_power_on_time_delay;
	messageOut->post_launch_power_on_time_delay_uncertainty = messageIn.post_launch_power_on_time_delay_uncertainty;
}

// Topic to Msg_1003
void convert(hg_nav_node::Msg_1003 messageIn, Msg_1003 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->anti_jam_selection = messageIn.anti_jam_selection;
	messageOut->gps_antenna_spin = messageIn.gps_antenna_spin;
	messageOut->constellation_type = messageIn.constellation_type;
	messageOut->acquisition_state_selection = messageIn.acquisition_state_selection;
	messageOut->ionosphere_corrections = messageIn.ionosphere_corrections;
	messageOut->post_launch_auto_transition_to_trk = messageIn.post_launch_auto_transition_to_trk;
	messageOut->set_back_shock = messageIn.set_back_shock;
	messageOut->position_uncertainty = messageIn.position_uncertainty;
	messageOut->velocity_uncertainty = messageIn.velocity_uncertainty;
	messageOut->time_uncertainty = messageIn.time_uncertainty;
	messageOut->number_of_acquisition_attempts = messageIn.number_of_acquisition_attempts;
	messageOut->gps_oscillator_calibration_jitter = messageIn.gps_oscillator_calibration_jitter;
	messageOut->gps_oscillator_calibration = messageIn.gps_oscillator_calibration;
	messageOut->gps_code_select = messageIn.gps_code_select;
	messageOut->gps_antenna_frequency_cmd = messageIn.gps_antenna_frequency_cmd;
	messageOut->gps_svid_deselect_1_4 = messageIn.gps_svid_deselect_1_4;
	messageOut->gps_svid_deselect_1_4_2 = messageIn.gps_svid_deselect_1_4_2;
	messageOut->pre_launch_temperature_ramp = messageIn.pre_launch_temperature_ramp;
	messageOut->post_launch_temperature_ramp = messageIn.post_launch_temperature_ramp;
	messageOut->post_launch_power_on_time_delay = messageIn.post_launch_power_on_time_delay;
	messageOut->post_launch_power_on_time_delay_uncertainty = messageIn.post_launch_power_on_time_delay_uncertainty;
}

void Msg_1003_sub_callback(const hg_nav_node::Msg_1003::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1003 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1003 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1003 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
