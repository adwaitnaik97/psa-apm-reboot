#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2002.h>
hg_nav_node::Msg_2002 msgStruct_2002;

bool Msg_2002_pub_initialized = false;

ros::Publisher Msg_2002_pub;
void init_2002(ros::NodeHandle * n){
	Msg_2002_pub = n->advertise<hg_nav_node::Msg_2002>(MSG_2002_PATH, 5);
	Msg_2002_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2002_PATH);
	return;
}

void stop_2002(void){
	Msg_2002_pub.shutdown();
	Msg_2002_pub_initialized = false;
	ROS_INFO("0x2002 stopped");
	return;
}

// Msg_2002 to Topic
void convert(Msg_2002 messageIn, hg_nav_node::Msg_2002 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GPS_Receiver_Application_Software[index] = messageIn.GPS_Receiver_Application_Software[index];
	}

}

// Topic to Msg_2002
void convert(hg_nav_node::Msg_2002 messageIn, Msg_2002 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GPS_Receiver_Application_Software[index] = messageIn.GPS_Receiver_Application_Software[index];
	}

}

void Msg_2002_pub_callback(uint8_t * buffer)
{
	Msg_2002 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2002 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2002);
	ROS_DEBUG("Message 0x2002 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2002_pub_initialized == false){
		init_2002(getRosHandle());}
	// Publish the message
	Msg_2002_pub.publish(msgStruct_2002);
	return;
}
