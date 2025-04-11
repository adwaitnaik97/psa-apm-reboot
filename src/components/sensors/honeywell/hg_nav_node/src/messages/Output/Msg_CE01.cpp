#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_CE01.h>
hg_nav_node::Msg_CE01 msgStruct_CE01;

bool Msg_CE01_pub_initialized = false;

ros::Publisher Msg_CE01_pub;
void init_CE01(ros::NodeHandle * n){
	Msg_CE01_pub = n->advertise<hg_nav_node::Msg_CE01>(MSG_CE01_PATH, 5);
	Msg_CE01_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_CE01_PATH);
	return;
}

void stop_CE01(void){
	Msg_CE01_pub.shutdown();
	Msg_CE01_pub_initialized = false;
	ROS_INFO("0xCE01 stopped");
	return;
}

// Msg_CE01 to Topic
void convert(Msg_CE01 messageIn, hg_nav_node::Msg_CE01 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->PortId = messageIn.PortId;
	messageOut->ASCII_String_Length = messageIn.ASCII_String_Length;
	for (unsigned int index = 0; index < 360; index++)
	{
		messageOut->ASCII_String[index] = messageIn.ASCII_String[index];
	}

}

// Topic to Msg_CE01
void convert(hg_nav_node::Msg_CE01 messageIn, Msg_CE01 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->PortId = messageIn.PortId;
	messageOut->ASCII_String_Length = messageIn.ASCII_String_Length;	for (unsigned int index = 0; index < 360; index++)
	{
		messageOut->ASCII_String[index] = messageIn.ASCII_String[index];
	}

}

void Msg_CE01_pub_callback(uint8_t * buffer)
{
	Msg_CE01 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xCE01 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_CE01);
	ROS_DEBUG("Message 0xCE01 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_CE01_pub_initialized == false){
		init_CE01(getRosHandle());}
	// Publish the message
	Msg_CE01_pub.publish(msgStruct_CE01);
	return;
}
