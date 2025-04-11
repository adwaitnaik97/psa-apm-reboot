#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_FE02.h>
hg_nav_node::Msg_FE02 msgStruct_FE02;

bool Msg_FE02_pub_initialized = false;

ros::Publisher Msg_FE02_pub;
void init_FE02(ros::NodeHandle * n){
	Msg_FE02_pub = n->advertise<hg_nav_node::Msg_FE02>(MSG_FE02_PATH, 5);
	Msg_FE02_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_FE02_PATH);
	return;
}

void stop_FE02(void){
	Msg_FE02_pub.shutdown();
	Msg_FE02_pub_initialized = false;
	ROS_INFO("0xFE02 stopped");
	return;
}

// Msg_FE02 to Topic
void convert(Msg_FE02 messageIn, hg_nav_node::Msg_FE02 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 348; index++)
	{
		messageOut->Payload[index] = messageIn.Payload[index];
	}

}

// Topic to Msg_FE02
void convert(hg_nav_node::Msg_FE02 messageIn, Msg_FE02 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 348; index++)
	{
		messageOut->Payload[index] = messageIn.Payload[index];
	}

}

void Msg_FE02_pub_callback(uint8_t * buffer)
{
	Msg_FE02 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xFE02 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_FE02);
	ROS_DEBUG("Message 0xFE02 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_FE02_pub_initialized == false){
		init_FE02(getRosHandle());}
	// Publish the message
	Msg_FE02_pub.publish(msgStruct_FE02);
	return;
}
