#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6724.h>
hg_nav_node::Msg_6724 msgStruct_6724;

bool Msg_6724_pub_initialized = false;

ros::Publisher Msg_6724_pub;
void init_6724(ros::NodeHandle * n){
	Msg_6724_pub = n->advertise<hg_nav_node::Msg_6724>(MSG_6724_PATH, 5);
	Msg_6724_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6724_PATH);
	return;
}

void stop_6724(void){
	Msg_6724_pub.shutdown();
	Msg_6724_pub_initialized = false;
	ROS_INFO("0x6724 stopped");
	return;
}

// Msg_6724 to Topic
void convert(Msg_6724 messageIn, hg_nav_node::Msg_6724 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam0[index] = messageIn.velBeam0[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam1[index] = messageIn.velBeam1[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam2[index] = messageIn.velBeam2[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam3[index] = messageIn.velBeam3[index];
	}

}

// Topic to Msg_6724
void convert(hg_nav_node::Msg_6724 messageIn, Msg_6724 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam0[index] = messageIn.velBeam0[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam1[index] = messageIn.velBeam1[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam2[index] = messageIn.velBeam2[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->velBeam3[index] = messageIn.velBeam3[index];
	}

}

void Msg_6724_pub_callback(uint8_t * buffer)
{
	Msg_6724 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6724 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6724);
	ROS_DEBUG("Message 0x6724 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6724_pub_initialized == false){
		init_6724(getRosHandle());}
	// Publish the message
	Msg_6724_pub.publish(msgStruct_6724);
	return;
}
