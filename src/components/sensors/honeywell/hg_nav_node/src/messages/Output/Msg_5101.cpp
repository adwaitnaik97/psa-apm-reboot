#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"
#include <vector> //Repeat Var

// Include custom message
#include <hg_nav_node/Msg_5101.h>
#include <hg_nav_node/constellation_enum_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/observation_t.h>
hg_nav_node::Msg_5101 msgStruct_5101;

bool Msg_5101_pub_initialized = false;

ros::Publisher Msg_5101_pub;
void init_5101(ros::NodeHandle * n){
	Msg_5101_pub = n->advertise<hg_nav_node::Msg_5101>(MSG_5101_PATH, 5);
	Msg_5101_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5101_PATH);
	return;
}

void stop_5101(void){
	Msg_5101_pub.shutdown();
	Msg_5101_pub_initialized = false;
	ROS_INFO("0x5101 stopped");
	return;
}

// Msg_5101 to Topic
void convert(Msg_5101 messageIn, hg_nav_node::Msg_5101 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->Num_Observations = messageIn.Num_Observations;

	//Variable lenght field
	messageOut->observation.resize(messageIn.get_observation_size());
	for (unsigned int i = 0; i < messageIn.get_observation_size(); i++){

	messageOut->observation[i].prn_number = messageIn.observation[i].prn_number;
	messageOut->observation[i].glofreq = messageIn.observation[i].glofreq;
	messageOut->observation[i].pseudorange = messageIn.observation[i].pseudorange;
	messageOut->observation[i].pseudorange_stdv = messageIn.observation[i].pseudorange_stdv;
	messageOut->observation[i].carrier_phase = messageIn.observation[i].carrier_phase;
	messageOut->observation[i].carrier_phase_stdv = messageIn.observation[i].carrier_phase_stdv;
	messageOut->observation[i].doppler_freq = messageIn.observation[i].doppler_freq;
	messageOut->observation[i].carrier_to_noise = messageIn.observation[i].carrier_to_noise;
	messageOut->observation[i].locktime = messageIn.observation[i].locktime;
	messageOut->observation[i].tracking_state = messageIn.observation[i].tracking_state;
	messageOut->observation[i].sv_channel_number = messageIn.observation[i].sv_channel_number;
	messageOut->observation[i].phase_lock = messageIn.observation[i].phase_lock;
	messageOut->observation[i].parity_known = messageIn.observation[i].parity_known;
	messageOut->observation[i].code_lock = messageIn.observation[i].code_lock;
	messageOut->observation[i].correlator_type = messageIn.observation[i].correlator_type;
	messageOut->observation[i].system.value = static_cast<uint8_t>(messageIn.observation[i].system);
	messageOut->observation[i].grouping = messageIn.observation[i].grouping;
	messageOut->observation[i].signal_type = messageIn.observation[i].signal_type;
	messageOut->observation[i].primary_L1_channel = messageIn.observation[i].primary_L1_channel;
	messageOut->observation[i].carrier_phase_meas = messageIn.observation[i].carrier_phase_meas;
	messageOut->observation[i].digital_filter = messageIn.observation[i].digital_filter;
	messageOut->observation[i].PRN_lock_flag = messageIn.observation[i].PRN_lock_flag;
	messageOut->observation[i].channel_assignment = messageIn.observation[i].channel_assignment;
	messageOut->observation[i].prn_number = messageIn.observation[i].prn_number;
	messageOut->observation[i].glofreq = messageIn.observation[i].glofreq;
	messageOut->observation[i].pseudorange = messageIn.observation[i].pseudorange;
	messageOut->observation[i].pseudorange_stdv = messageIn.observation[i].pseudorange_stdv;
	messageOut->observation[i].carrier_phase = messageIn.observation[i].carrier_phase;
	messageOut->observation[i].carrier_phase_stdv = messageIn.observation[i].carrier_phase_stdv;
	messageOut->observation[i].doppler_freq = messageIn.observation[i].doppler_freq;
	messageOut->observation[i].carrier_to_noise = messageIn.observation[i].carrier_to_noise;
	messageOut->observation[i].locktime = messageIn.observation[i].locktime;
	messageOut->observation[i].tracking_state = messageIn.observation[i].tracking_state;
	messageOut->observation[i].sv_channel_number = messageIn.observation[i].sv_channel_number;
	messageOut->observation[i].phase_lock = messageIn.observation[i].phase_lock;
	messageOut->observation[i].parity_known = messageIn.observation[i].parity_known;
	messageOut->observation[i].code_lock = messageIn.observation[i].code_lock;
	messageOut->observation[i].correlator_type = messageIn.observation[i].correlator_type;
	messageOut->observation[i].system.value = static_cast<uint8_t>(messageIn.observation[i].system);
	messageOut->observation[i].grouping = messageIn.observation[i].grouping;
	messageOut->observation[i].signal_type = messageIn.observation[i].signal_type;
	messageOut->observation[i].primary_L1_channel = messageIn.observation[i].primary_L1_channel;
	messageOut->observation[i].carrier_phase_meas = messageIn.observation[i].carrier_phase_meas;
	messageOut->observation[i].digital_filter = messageIn.observation[i].digital_filter;
	messageOut->observation[i].PRN_lock_flag = messageIn.observation[i].PRN_lock_flag;
	messageOut->observation[i].channel_assignment = messageIn.observation[i].channel_assignment;
	}
}

// Topic to Msg_5101
void convert(hg_nav_node::Msg_5101 messageIn, Msg_5101 * messageOut)
{
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->Num_Observations = messageIn.Num_Observations;

	//Variable lenght field
	uint32_t observation_size = (messageIn.observation.size()<=messageOut->get_observation_size())?messageIn.observation.size():messageOut->get_observation_size();
	for (unsigned int i = 0; i < observation_size; i++){

	messageOut->observation[i].prn_number = messageIn.observation[i].prn_number;
	messageOut->observation[i].glofreq = messageIn.observation[i].glofreq;
	messageOut->observation[i].pseudorange = messageIn.observation[i].pseudorange;
	messageOut->observation[i].pseudorange_stdv = messageIn.observation[i].pseudorange_stdv;
	messageOut->observation[i].carrier_phase = messageIn.observation[i].carrier_phase;
	messageOut->observation[i].carrier_phase_stdv = messageIn.observation[i].carrier_phase_stdv;
	messageOut->observation[i].doppler_freq = messageIn.observation[i].doppler_freq;
	messageOut->observation[i].carrier_to_noise = messageIn.observation[i].carrier_to_noise;
	messageOut->observation[i].locktime = messageIn.observation[i].locktime;
	messageOut->observation[i].tracking_state = messageIn.observation[i].tracking_state;
	messageOut->observation[i].sv_channel_number = messageIn.observation[i].sv_channel_number;
	messageOut->observation[i].phase_lock = messageIn.observation[i].phase_lock;
	messageOut->observation[i].parity_known = messageIn.observation[i].parity_known;
	messageOut->observation[i].code_lock = messageIn.observation[i].code_lock;
	messageOut->observation[i].correlator_type = messageIn.observation[i].correlator_type;
	messageOut->observation[i].system = static_cast<constellation_enum_t>(messageIn.observation[i].system.value);
	messageOut->observation[i].grouping = messageIn.observation[i].grouping;
	messageOut->observation[i].signal_type = messageIn.observation[i].signal_type;
	messageOut->observation[i].primary_L1_channel = messageIn.observation[i].primary_L1_channel;
	messageOut->observation[i].carrier_phase_meas = messageIn.observation[i].carrier_phase_meas;
	messageOut->observation[i].digital_filter = messageIn.observation[i].digital_filter;
	messageOut->observation[i].PRN_lock_flag = messageIn.observation[i].PRN_lock_flag;
	messageOut->observation[i].channel_assignment = messageIn.observation[i].channel_assignment;
	messageOut->observation[i].prn_number = messageIn.observation[i].prn_number;
	messageOut->observation[i].glofreq = messageIn.observation[i].glofreq;
	messageOut->observation[i].pseudorange = messageIn.observation[i].pseudorange;
	messageOut->observation[i].pseudorange_stdv = messageIn.observation[i].pseudorange_stdv;
	messageOut->observation[i].carrier_phase = messageIn.observation[i].carrier_phase;
	messageOut->observation[i].carrier_phase_stdv = messageIn.observation[i].carrier_phase_stdv;
	messageOut->observation[i].doppler_freq = messageIn.observation[i].doppler_freq;
	messageOut->observation[i].carrier_to_noise = messageIn.observation[i].carrier_to_noise;
	messageOut->observation[i].locktime = messageIn.observation[i].locktime;
	messageOut->observation[i].tracking_state = messageIn.observation[i].tracking_state;
	messageOut->observation[i].sv_channel_number = messageIn.observation[i].sv_channel_number;
	messageOut->observation[i].phase_lock = messageIn.observation[i].phase_lock;
	messageOut->observation[i].parity_known = messageIn.observation[i].parity_known;
	messageOut->observation[i].code_lock = messageIn.observation[i].code_lock;
	messageOut->observation[i].correlator_type = messageIn.observation[i].correlator_type;
	messageOut->observation[i].system = static_cast<constellation_enum_t>(messageIn.observation[i].system.value);
	messageOut->observation[i].grouping = messageIn.observation[i].grouping;
	messageOut->observation[i].signal_type = messageIn.observation[i].signal_type;
	messageOut->observation[i].primary_L1_channel = messageIn.observation[i].primary_L1_channel;
	messageOut->observation[i].carrier_phase_meas = messageIn.observation[i].carrier_phase_meas;
	messageOut->observation[i].digital_filter = messageIn.observation[i].digital_filter;
	messageOut->observation[i].PRN_lock_flag = messageIn.observation[i].PRN_lock_flag;
	messageOut->observation[i].channel_assignment = messageIn.observation[i].channel_assignment;
	}
}

void Msg_5101_pub_callback(uint8_t * buffer)
{
	Msg_5101 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5101 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5101);
	ROS_DEBUG("Message 0x5101 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5101_pub_initialized == false){
		init_5101(getRosHandle());}
	// Publish the message
	Msg_5101_pub.publish(msgStruct_5101);
	return;
}
