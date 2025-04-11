#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F4.h>
hg_node::Msg_F4 msgStruct_F4;

bool Msg_F4_pub_initialized = false;

ros::Publisher Msg_F4_pub;
void init_F4(ros::NodeHandle * n){
	Msg_F4_pub = n->advertise<hg_node::Msg_F4>(MSG_F4_PATH, 5);
	Msg_F4_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F4_PATH);
	return;
}

void stop_F4(void){
	if (Msg_F4_pub_initialized){
		Msg_F4_pub.shutdown();
		Msg_F4_pub_initialized = false;
		ROS_INFO("0xF4 stopped");
	}
	return;
}

// Msg_F4 to Topic
void convert(Msg_F4 messageIn, hg_node::Msg_F4 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->baud_rate = messageIn.baud_rate;
	messageOut->bytes_transmitted = messageIn.bytes_transmitted;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F4
void convert(hg_node::Msg_F4 messageIn, Msg_F4 * messageOut)
{
	messageOut->baud_rate = messageIn.baud_rate;
	messageOut->bytes_transmitted = messageIn.bytes_transmitted;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F4_pub_callback(uint8_t * buffer)
{
	Msg_F4 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF4 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F4);
	ROS_DEBUG("Message 0xF4 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F4_pub_initialized == false){
		init_F4(getRosHandle());}
	// Publish the message
	Msg_F4_pub.publish(msgStruct_F4);
	return;
}
