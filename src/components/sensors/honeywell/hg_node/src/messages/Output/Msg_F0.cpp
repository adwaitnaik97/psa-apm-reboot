#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F0.h>
hg_node::Msg_F0 msgStruct_F0;

bool Msg_F0_pub_initialized = false;

ros::Publisher Msg_F0_pub;
void init_F0(ros::NodeHandle * n){
	Msg_F0_pub = n->advertise<hg_node::Msg_F0>(MSG_F0_PATH, 5);
	Msg_F0_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F0_PATH);
	return;
}

void stop_F0(void){
	if (Msg_F0_pub_initialized){
		Msg_F0_pub.shutdown();
		Msg_F0_pub_initialized = false;
		ROS_INFO("0xF0 stopped");
	}
	return;
}

// Msg_F0 to Topic
void convert(Msg_F0 messageIn, hg_node::Msg_F0 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_type[index] = messageIn.device_type[index];
	}

	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_config[index] = messageIn.device_config[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F0
void convert(hg_node::Msg_F0 messageIn, Msg_F0 * messageOut)
{
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_type[index] = messageIn.device_type[index];
	}

	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_config[index] = messageIn.device_config[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F0_pub_callback(uint8_t * buffer)
{
	Msg_F0 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF0 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F0);
	ROS_DEBUG("Message 0xF0 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F0_pub_initialized == false){
		init_F0(getRosHandle());}
	// Publish the message
	Msg_F0_pub.publish(msgStruct_F0);
	return;
}
