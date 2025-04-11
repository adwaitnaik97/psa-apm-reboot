#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_9910.h>
hg_nav_node::Msg_9910 msgStruct_9910;

bool Msg_9910_pub_initialized = false;

ros::Publisher Msg_9910_pub;
void init_9910(ros::NodeHandle * n){
	Msg_9910_pub = n->advertise<hg_nav_node::Msg_9910>(MSG_9910_PATH, 5);
	Msg_9910_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_9910_PATH);
	return;
}

void stop_9910(void){
	Msg_9910_pub.shutdown();
	Msg_9910_pub_initialized = false;
	ROS_INFO("0x9910 stopped");
	return;
}

// Msg_9910 to Topic
void convert(Msg_9910 messageIn, hg_nav_node::Msg_9910 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->own_usage_rate_0 = messageIn.own_usage_rate_0;
	messageOut->own_usage_rate_1 = messageIn.own_usage_rate_1;
	messageOut->own_usage_rate_2 = messageIn.own_usage_rate_2;
	messageOut->own_usage_rate_3 = messageIn.own_usage_rate_3;
	messageOut->own_usage_rate_4 = messageIn.own_usage_rate_4;
	messageOut->own_usage_rate_5 = messageIn.own_usage_rate_5;
	messageOut->own_usage_rate_6 = messageIn.own_usage_rate_6;
	messageOut->own_usage_rate_7 = messageIn.own_usage_rate_7;
	messageOut->total_usage_rate_0 = messageIn.total_usage_rate_0;
	messageOut->total_usage_rate_1 = messageIn.total_usage_rate_1;
	messageOut->total_usage_rate_2 = messageIn.total_usage_rate_2;
	messageOut->total_usage_rate_3 = messageIn.total_usage_rate_3;
	messageOut->total_usage_rate_4 = messageIn.total_usage_rate_4;
	messageOut->total_usage_rate_5 = messageIn.total_usage_rate_5;
	messageOut->total_usage_rate_6 = messageIn.total_usage_rate_6;
	messageOut->total_usage_rate_7 = messageIn.total_usage_rate_7;
	messageOut->max_usage_rate_0 = messageIn.max_usage_rate_0;
	messageOut->max_usage_rate_1 = messageIn.max_usage_rate_1;
	messageOut->max_usage_rate_2 = messageIn.max_usage_rate_2;
	messageOut->max_usage_rate_3 = messageIn.max_usage_rate_3;
	messageOut->max_usage_rate_4 = messageIn.max_usage_rate_4;
	messageOut->max_usage_rate_5 = messageIn.max_usage_rate_5;
	messageOut->max_usage_rate_6 = messageIn.max_usage_rate_6;
	messageOut->max_usage_rate_7 = messageIn.max_usage_rate_7;
	messageOut->total_usage = messageIn.total_usage;
	messageOut->max_usage = messageIn.max_usage;
}

// Topic to Msg_9910
void convert(hg_nav_node::Msg_9910 messageIn, Msg_9910 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->own_usage_rate_0 = messageIn.own_usage_rate_0;
	messageOut->own_usage_rate_1 = messageIn.own_usage_rate_1;
	messageOut->own_usage_rate_2 = messageIn.own_usage_rate_2;
	messageOut->own_usage_rate_3 = messageIn.own_usage_rate_3;
	messageOut->own_usage_rate_4 = messageIn.own_usage_rate_4;
	messageOut->own_usage_rate_5 = messageIn.own_usage_rate_5;
	messageOut->own_usage_rate_6 = messageIn.own_usage_rate_6;
	messageOut->own_usage_rate_7 = messageIn.own_usage_rate_7;
	messageOut->total_usage_rate_0 = messageIn.total_usage_rate_0;
	messageOut->total_usage_rate_1 = messageIn.total_usage_rate_1;
	messageOut->total_usage_rate_2 = messageIn.total_usage_rate_2;
	messageOut->total_usage_rate_3 = messageIn.total_usage_rate_3;
	messageOut->total_usage_rate_4 = messageIn.total_usage_rate_4;
	messageOut->total_usage_rate_5 = messageIn.total_usage_rate_5;
	messageOut->total_usage_rate_6 = messageIn.total_usage_rate_6;
	messageOut->total_usage_rate_7 = messageIn.total_usage_rate_7;
	messageOut->max_usage_rate_0 = messageIn.max_usage_rate_0;
	messageOut->max_usage_rate_1 = messageIn.max_usage_rate_1;
	messageOut->max_usage_rate_2 = messageIn.max_usage_rate_2;
	messageOut->max_usage_rate_3 = messageIn.max_usage_rate_3;
	messageOut->max_usage_rate_4 = messageIn.max_usage_rate_4;
	messageOut->max_usage_rate_5 = messageIn.max_usage_rate_5;
	messageOut->max_usage_rate_6 = messageIn.max_usage_rate_6;
	messageOut->max_usage_rate_7 = messageIn.max_usage_rate_7;
	messageOut->total_usage = messageIn.total_usage;
	messageOut->max_usage = messageIn.max_usage;
}

void Msg_9910_pub_callback(uint8_t * buffer)
{
	Msg_9910 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x9910 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_9910);
	ROS_DEBUG("Message 0x9910 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_9910_pub_initialized == false){
		init_9910(getRosHandle());}
	// Publish the message
	Msg_9910_pub.publish(msgStruct_9910);
	return;
}
