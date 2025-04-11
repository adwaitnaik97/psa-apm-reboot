#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_9900.h>
hg_nav_node::Msg_9900 msgStruct_9900;

bool Msg_9900_pub_initialized = false;

ros::Publisher Msg_9900_pub;
void init_9900(ros::NodeHandle * n){
	Msg_9900_pub = n->advertise<hg_nav_node::Msg_9900>(MSG_9900_PATH, 5);
	Msg_9900_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_9900_PATH);
	return;
}

void stop_9900(void){
	Msg_9900_pub.shutdown();
	Msg_9900_pub_initialized = false;
	ROS_INFO("0x9900 stopped");
	return;
}

// Msg_9900 to Topic
void convert(Msg_9900 messageIn, hg_nav_node::Msg_9900 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 160; index++)
	{
		messageOut->ASCII_String[index] = messageIn.ASCII_String[index];
	}

}

// Topic to Msg_9900
void convert(hg_nav_node::Msg_9900 messageIn, Msg_9900 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 160; index++)
	{
		messageOut->ASCII_String[index] = messageIn.ASCII_String[index];
	}

}

void Msg_9900_pub_callback(uint8_t * buffer)
{
	Msg_9900 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x9900 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_9900);
	ROS_DEBUG("Message 0x9900 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_9900_pub_initialized == false){
		init_9900(getRosHandle());}
	// Publish the message
	Msg_9900_pub.publish(msgStruct_9900);
	return;
}
