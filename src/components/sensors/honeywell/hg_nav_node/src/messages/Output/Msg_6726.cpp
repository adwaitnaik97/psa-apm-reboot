#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6726.h>
hg_nav_node::Msg_6726 msgStruct_6726;

bool Msg_6726_pub_initialized = false;

ros::Publisher Msg_6726_pub;
void init_6726(ros::NodeHandle * n){
	Msg_6726_pub = n->advertise<hg_nav_node::Msg_6726>(MSG_6726_PATH, 5);
	Msg_6726_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6726_PATH);
	return;
}

void stop_6726(void){
	Msg_6726_pub.shutdown();
	Msg_6726_pub_initialized = false;
	ROS_INFO("0x6726 stopped");
	return;
}

// Msg_6726 to Topic
void convert(Msg_6726 messageIn, hg_nav_node::Msg_6726 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam0[index] = messageIn.corBeam0[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam1[index] = messageIn.corBeam1[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam2[index] = messageIn.corBeam2[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam3[index] = messageIn.corBeam3[index];
	}

}

// Topic to Msg_6726
void convert(hg_nav_node::Msg_6726 messageIn, Msg_6726 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam0[index] = messageIn.corBeam0[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam1[index] = messageIn.corBeam1[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam2[index] = messageIn.corBeam2[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->corBeam3[index] = messageIn.corBeam3[index];
	}

}

void Msg_6726_pub_callback(uint8_t * buffer)
{
	Msg_6726 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6726 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6726);
	ROS_DEBUG("Message 0x6726 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6726_pub_initialized == false){
		init_6726(getRosHandle());}
	// Publish the message
	Msg_6726_pub.publish(msgStruct_6726);
	return;
}
