#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_FB.h>
hg_node::Msg_FB msgStruct_FB;

bool Msg_FB_pub_initialized = false;

ros::Publisher Msg_FB_pub;
void init_FB(ros::NodeHandle * n){
	Msg_FB_pub = n->advertise<hg_node::Msg_FB>(MSG_FB_PATH, 5);
	Msg_FB_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_FB_PATH);
	return;
}

void stop_FB(void){
	if (Msg_FB_pub_initialized){
		Msg_FB_pub.shutdown();
		Msg_FB_pub_initialized = false;
		ROS_INFO("0xFB stopped");
	}
	return;
}

// Msg_FB to Topic
void convert(Msg_FB messageIn, hg_node::Msg_FB * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->crc_fail = messageIn.crc_fail;
	messageOut->hardware_fail = messageIn.hardware_fail;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_FB
void convert(hg_node::Msg_FB messageIn, Msg_FB * messageOut)
{
	messageOut->crc_fail = messageIn.crc_fail;
	messageOut->hardware_fail = messageIn.hardware_fail;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_FB_pub_callback(uint8_t * buffer)
{
	Msg_FB Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xFB deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_FB);
	ROS_DEBUG("Message 0xFB Received");

	// Initialize Publisher if not initialized yet
	if (Msg_FB_pub_initialized == false){
		init_FB(getRosHandle());}
	// Publish the message
	Msg_FB_pub.publish(msgStruct_FB);
	return;
}
