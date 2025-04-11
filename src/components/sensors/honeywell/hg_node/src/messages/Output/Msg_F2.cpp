#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F2.h>
hg_node::Msg_F2 msgStruct_F2;

bool Msg_F2_pub_initialized = false;

ros::Publisher Msg_F2_pub;
void init_F2(ros::NodeHandle * n){
	Msg_F2_pub = n->advertise<hg_node::Msg_F2>(MSG_F2_PATH, 5);
	Msg_F2_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F2_PATH);
	return;
}

void stop_F2(void){
	if (Msg_F2_pub_initialized){
		Msg_F2_pub.shutdown();
		Msg_F2_pub_initialized = false;
		ROS_INFO("0xF2 stopped");
	}
	return;
}

// Msg_F2 to Topic
void convert(Msg_F2 messageIn, hg_node::Msg_F2 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->part_number = messageIn.part_number;
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->hardware_version[index] = messageIn.hardware_version[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F2
void convert(hg_node::Msg_F2 messageIn, Msg_F2 * messageOut)
{
	messageOut->part_number = messageIn.part_number;
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->hardware_version[index] = messageIn.hardware_version[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F2_pub_callback(uint8_t * buffer)
{
	Msg_F2 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF2 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F2);
	ROS_DEBUG("Message 0xF2 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F2_pub_initialized == false){
		init_F2(getRosHandle());}
	// Publish the message
	Msg_F2_pub.publish(msgStruct_F2);
	return;
}
