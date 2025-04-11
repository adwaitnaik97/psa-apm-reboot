#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2021.h>
hg_nav_node::Msg_2021 msgStruct_2021;

bool Msg_2021_pub_initialized = false;

ros::Publisher Msg_2021_pub;
void init_2021(ros::NodeHandle * n){
	Msg_2021_pub = n->advertise<hg_nav_node::Msg_2021>(MSG_2021_PATH, 5);
	Msg_2021_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2021_PATH);
	return;
}

void stop_2021(void){
	Msg_2021_pub.shutdown();
	Msg_2021_pub_initialized = false;
	ROS_INFO("0x2021 stopped");
	return;
}

// Msg_2021 to Topic
void convert(Msg_2021 messageIn, hg_nav_node::Msg_2021 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->initialization_status_1 = messageIn.initialization_status_1;
	messageOut->initialization_status_2 = messageIn.initialization_status_2;
	messageOut->number_tm_pps_pulses_rcvd = messageIn.number_tm_pps_pulses_rcvd;
	messageOut->number_time_msgs_rcvd = messageIn.number_time_msgs_rcvd;
	messageOut->clock_calibration_status = messageIn.clock_calibration_status;
	messageOut->oscillator_drift = messageIn.oscillator_drift;
	messageOut->almanac_database = messageIn.almanac_database;
	messageOut->ephemeris_database = messageIn.ephemeris_database;
	messageOut->iono_database = messageIn.iono_database;
	messageOut->subframe_4_database = messageIn.subframe_4_database;
	messageOut->subframe_5_database = messageIn.subframe_5_database;
}

// Topic to Msg_2021
void convert(hg_nav_node::Msg_2021 messageIn, Msg_2021 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->initialization_status_1 = messageIn.initialization_status_1;
	messageOut->initialization_status_2 = messageIn.initialization_status_2;
	messageOut->number_tm_pps_pulses_rcvd = messageIn.number_tm_pps_pulses_rcvd;
	messageOut->number_time_msgs_rcvd = messageIn.number_time_msgs_rcvd;
	messageOut->clock_calibration_status = messageIn.clock_calibration_status;
	messageOut->oscillator_drift = messageIn.oscillator_drift;
	messageOut->almanac_database = messageIn.almanac_database;
	messageOut->ephemeris_database = messageIn.ephemeris_database;
	messageOut->iono_database = messageIn.iono_database;
	messageOut->subframe_4_database = messageIn.subframe_4_database;
	messageOut->subframe_5_database = messageIn.subframe_5_database;
}

void Msg_2021_pub_callback(uint8_t * buffer)
{
	Msg_2021 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2021 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2021);
	ROS_DEBUG("Message 0x2021 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2021_pub_initialized == false){
		init_2021(getRosHandle());}
	// Publish the message
	Msg_2021_pub.publish(msgStruct_2021);
	return;
}
