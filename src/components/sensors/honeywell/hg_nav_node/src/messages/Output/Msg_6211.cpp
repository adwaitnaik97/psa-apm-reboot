#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6211.h>
#include <hg_nav_node/event_out_mark_t.h>
hg_nav_node::Msg_6211 msgStruct_6211;

bool Msg_6211_pub_initialized = false;

ros::Publisher Msg_6211_pub;
void init_6211(ros::NodeHandle * n){
	Msg_6211_pub = n->advertise<hg_nav_node::Msg_6211>(MSG_6211_PATH, 5);
	Msg_6211_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6211_PATH);
	return;
}

void stop_6211(void){
	Msg_6211_pub.shutdown();
	Msg_6211_pub_initialized = false;
	ROS_INFO("0x6211 stopped");
	return;
}

// Msg_6211 to Topic
void convert(Msg_6211 messageIn, hg_nav_node::Msg_6211 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->markPort.value = static_cast<uint8_t>(messageIn.markPort);
	messageOut->system_time_of_event_in = messageIn.system_time_of_event_in;
	messageOut->gps_time_of_event_in = messageIn.gps_time_of_event_in;
	messageOut->gps_week_of_event_in = messageIn.gps_week_of_event_in;
}

// Topic to Msg_6211
void convert(hg_nav_node::Msg_6211 messageIn, Msg_6211 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->markPort = static_cast<event_out_mark_t>(messageIn.markPort.value);
	messageOut->system_time_of_event_in = messageIn.system_time_of_event_in;
	messageOut->gps_time_of_event_in = messageIn.gps_time_of_event_in;
	messageOut->gps_week_of_event_in = messageIn.gps_week_of_event_in;
}

void Msg_6211_pub_callback(uint8_t * buffer)
{
	Msg_6211 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6211 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6211);
	ROS_DEBUG("Message 0x6211 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6211_pub_initialized == false){
		init_6211(getRosHandle());}
	// Publish the message
	Msg_6211_pub.publish(msgStruct_6211);
	return;
}
