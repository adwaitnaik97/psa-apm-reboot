#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F8.h>
hg_node::Msg_F8 msgStruct_F8;

bool Msg_F8_pub_initialized = false;

ros::Publisher Msg_F8_pub;
void init_F8(ros::NodeHandle * n){
	Msg_F8_pub = n->advertise<hg_node::Msg_F8>(MSG_F8_PATH, 5);
	Msg_F8_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F8_PATH);
	return;
}

void stop_F8(void){
	if (Msg_F8_pub_initialized){
		Msg_F8_pub.shutdown();
		Msg_F8_pub_initialized = false;
		ROS_INFO("0xF8 stopped");
	}
	return;
}

// Msg_F8 to Topic
void convert(Msg_F8 messageIn, hg_node::Msg_F8 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->device_current = messageIn.device_current;
	messageOut->device_voltage = messageIn.device_voltage;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F8
void convert(hg_node::Msg_F8 messageIn, Msg_F8 * messageOut)
{
	messageOut->device_current = messageIn.device_current;
	messageOut->device_voltage = messageIn.device_voltage;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F8_pub_callback(uint8_t * buffer)
{
	Msg_F8 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF8 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F8);
	ROS_DEBUG("Message 0xF8 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F8_pub_initialized == false){
		init_F8(getRosHandle());}
	// Publish the message
	Msg_F8_pub.publish(msgStruct_F8);
	return;
}
