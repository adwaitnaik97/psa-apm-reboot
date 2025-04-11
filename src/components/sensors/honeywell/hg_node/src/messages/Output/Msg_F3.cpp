#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F3.h>
hg_node::Msg_F3 msgStruct_F3;

bool Msg_F3_pub_initialized = false;

ros::Publisher Msg_F3_pub;
void init_F3(ros::NodeHandle * n){
	Msg_F3_pub = n->advertise<hg_node::Msg_F3>(MSG_F3_PATH, 5);
	Msg_F3_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F3_PATH);
	return;
}

void stop_F3(void){
	if (Msg_F3_pub_initialized){
		Msg_F3_pub.shutdown();
		Msg_F3_pub_initialized = false;
		ROS_INFO("0xF3 stopped");
	}
	return;
}

// Msg_F3 to Topic
void convert(Msg_F3 messageIn, hg_node::Msg_F3 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->firmware_major_version = messageIn.firmware_major_version;
	messageOut->firmware_minor_version = messageIn.firmware_minor_version;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F3
void convert(hg_node::Msg_F3 messageIn, Msg_F3 * messageOut)
{
	messageOut->firmware_major_version = messageIn.firmware_major_version;
	messageOut->firmware_minor_version = messageIn.firmware_minor_version;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F3_pub_callback(uint8_t * buffer)
{
	Msg_F3 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF3 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F3);
	ROS_DEBUG("Message 0xF3 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F3_pub_initialized == false){
		init_F3(getRosHandle());}
	// Publish the message
	Msg_F3_pub.publish(msgStruct_F3);
	return;
}
