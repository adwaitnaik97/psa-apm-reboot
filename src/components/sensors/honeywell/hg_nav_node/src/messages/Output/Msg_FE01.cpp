#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_FE01.h>
#include <hg_nav_node/msg_FE01_parameter_1_t.h>
#include <hg_nav_node/msg_FE01_write_fail_reason_t.h>
hg_nav_node::Msg_FE01 msgStruct_FE01;

bool Msg_FE01_pub_initialized = false;

ros::Publisher Msg_FE01_pub;
void init_FE01(ros::NodeHandle * n){
	Msg_FE01_pub = n->advertise<hg_nav_node::Msg_FE01>(MSG_FE01_PATH, 5);
	Msg_FE01_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_FE01_PATH);
	return;
}

void stop_FE01(void){
	Msg_FE01_pub.shutdown();
	Msg_FE01_pub_initialized = false;
	ROS_INFO("0xFE01 stopped");
	return;
}

// Msg_FE01 to Topic
void convert(Msg_FE01 messageIn, hg_nav_node::Msg_FE01 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Parameter_1.value = static_cast<uint8_t>(messageIn.Parameter_1);
	messageOut->Parameter_2 = messageIn.Parameter_2;
	messageOut->Parameter_3 = messageIn.Parameter_3;
	messageOut->Parameter_4 = messageIn.Parameter_4;
	for (unsigned int index = 0; index < 248; index++)
	{
		messageOut->Payload[index] = messageIn.Payload[index];
	}

}

// Topic to Msg_FE01
void convert(hg_nav_node::Msg_FE01 messageIn, Msg_FE01 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Parameter_1 = static_cast<msg_FE01_parameter_1_t>(messageIn.Parameter_1.value);
	messageOut->Parameter_2 = messageIn.Parameter_2;
	messageOut->Parameter_3 = messageIn.Parameter_3;
	messageOut->Parameter_4 = messageIn.Parameter_4;	for (unsigned int index = 0; index < 248; index++)
	{
		messageOut->Payload[index] = messageIn.Payload[index];
	}

}

void Msg_FE01_pub_callback(uint8_t * buffer)
{
	Msg_FE01 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xFE01 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_FE01);
	ROS_DEBUG("Message 0xFE01 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_FE01_pub_initialized == false){
		init_FE01(getRosHandle());}
	// Publish the message
	Msg_FE01_pub.publish(msgStruct_FE01);
	return;
}
