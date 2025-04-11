#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6725.h>
hg_nav_node::Msg_6725 msgStruct_6725;

bool Msg_6725_pub_initialized = false;

ros::Publisher Msg_6725_pub;
void init_6725(ros::NodeHandle * n){
	Msg_6725_pub = n->advertise<hg_nav_node::Msg_6725>(MSG_6725_PATH, 5);
	Msg_6725_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6725_PATH);
	return;
}

void stop_6725(void){
	Msg_6725_pub.shutdown();
	Msg_6725_pub_initialized = false;
	ROS_INFO("0x6725 stopped");
	return;
}

// Msg_6725 to Topic
void convert(Msg_6725 messageIn, hg_nav_node::Msg_6725 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam0[index] = messageIn.ampBeam0[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam1[index] = messageIn.ampBeam1[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam2[index] = messageIn.ampBeam2[index];
	}

	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam3[index] = messageIn.ampBeam3[index];
	}

}

// Topic to Msg_6725
void convert(hg_nav_node::Msg_6725 messageIn, Msg_6725 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam0[index] = messageIn.ampBeam0[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam1[index] = messageIn.ampBeam1[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam2[index] = messageIn.ampBeam2[index];
	}
	for (unsigned int index = 0; index < 30; index++)
	{
		messageOut->ampBeam3[index] = messageIn.ampBeam3[index];
	}

}

void Msg_6725_pub_callback(uint8_t * buffer)
{
	Msg_6725 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6725 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6725);
	ROS_DEBUG("Message 0x6725 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6725_pub_initialized == false){
		init_6725(getRosHandle());}
	// Publish the message
	Msg_6725_pub.publish(msgStruct_6725);
	return;
}
