#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F7.h>
hg_node::Msg_F7 msgStruct_F7;

bool Msg_F7_pub_initialized = false;

ros::Publisher Msg_F7_pub;
void init_F7(ros::NodeHandle * n){
	Msg_F7_pub = n->advertise<hg_node::Msg_F7>(MSG_F7_PATH, 5);
	Msg_F7_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F7_PATH);
	return;
}

void stop_F7(void){
	if (Msg_F7_pub_initialized){
		Msg_F7_pub.shutdown();
		Msg_F7_pub_initialized = false;
		ROS_INFO("0xF7 stopped");
	}
	return;
}

// Msg_F7 to Topic
void convert(Msg_F7 messageIn, hg_node::Msg_F7 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->hours_of_operation = messageIn.hours_of_operation;
	messageOut->boot_count = messageIn.boot_count;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F7
void convert(hg_node::Msg_F7 messageIn, Msg_F7 * messageOut)
{
	messageOut->hours_of_operation = messageIn.hours_of_operation;
	messageOut->boot_count = messageIn.boot_count;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F7_pub_callback(uint8_t * buffer)
{
	Msg_F7 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF7 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F7);
	ROS_DEBUG("Message 0xF7 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F7_pub_initialized == false){
		init_F7(getRosHandle());}
	// Publish the message
	Msg_F7_pub.publish(msgStruct_F7);
	return;
}
