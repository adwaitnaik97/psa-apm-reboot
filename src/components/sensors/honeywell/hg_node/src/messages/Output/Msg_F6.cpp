#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F6.h>
#include <hg_node/filter_config_t.h>
hg_node::Msg_F6 msgStruct_F6;

bool Msg_F6_pub_initialized = false;

ros::Publisher Msg_F6_pub;
void init_F6(ros::NodeHandle * n){
	Msg_F6_pub = n->advertise<hg_node::Msg_F6>(MSG_F6_PATH, 5);
	Msg_F6_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F6_PATH);
	return;
}

void stop_F6(void){
	if (Msg_F6_pub_initialized){
		Msg_F6_pub.shutdown();
		Msg_F6_pub_initialized = false;
		ROS_INFO("0xF6 stopped");
	}
	return;
}

// Msg_F6 to Topic
void convert(Msg_F6 messageIn, hg_node::Msg_F6 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->accel_range = messageIn.accel_range;

	messageOut->accel_config.cutoff_frequency = messageIn.accel_config.cutoff_frequency;
	messageOut->accel_config.enabled = messageIn.accel_config.enabled;
	messageOut->accel_sample_rate = messageIn.accel_sample_rate;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F6
void convert(hg_node::Msg_F6 messageIn, Msg_F6 * messageOut)
{
	messageOut->accel_range = messageIn.accel_range;

	messageOut->accel_config.cutoff_frequency = messageIn.accel_config.cutoff_frequency;
	messageOut->accel_config.enabled = messageIn.accel_config.enabled;
	messageOut->accel_sample_rate = messageIn.accel_sample_rate;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F6_pub_callback(uint8_t * buffer)
{
	Msg_F6 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF6 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F6);
	ROS_DEBUG("Message 0xF6 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F6_pub_initialized == false){
		init_F6(getRosHandle());}
	// Publish the message
	Msg_F6_pub.publish(msgStruct_F6);
	return;
}
