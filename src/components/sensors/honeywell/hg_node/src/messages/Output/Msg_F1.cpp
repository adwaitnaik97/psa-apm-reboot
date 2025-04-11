#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F1.h>
hg_node::Msg_F1 msgStruct_F1;

bool Msg_F1_pub_initialized = false;

ros::Publisher Msg_F1_pub;
void init_F1(ros::NodeHandle * n){
	Msg_F1_pub = n->advertise<hg_node::Msg_F1>(MSG_F1_PATH, 5);
	Msg_F1_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F1_PATH);
	return;
}

void stop_F1(void){
	if (Msg_F1_pub_initialized){
		Msg_F1_pub.shutdown();
		Msg_F1_pub_initialized = false;
		ROS_INFO("0xF1 stopped");
	}
	return;
}

// Msg_F1 to Topic
void convert(Msg_F1 messageIn, hg_node::Msg_F1 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->serial_number[index] = messageIn.serial_number[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F1
void convert(hg_node::Msg_F1 messageIn, Msg_F1 * messageOut)
{
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->serial_number[index] = messageIn.serial_number[index];
	}

	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F1_pub_callback(uint8_t * buffer)
{
	Msg_F1 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF1 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F1);
	ROS_DEBUG("Message 0xF1 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F1_pub_initialized == false){
		init_F1(getRosHandle());}
	// Publish the message
	Msg_F1_pub.publish(msgStruct_F1);
	return;
}
