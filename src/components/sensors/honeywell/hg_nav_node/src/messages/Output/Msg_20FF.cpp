#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_20FF.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_20FF msgStruct_20FF;

bool Msg_20FF_pub_initialized = false;

ros::Publisher Msg_20FF_pub;
void init_20FF(ros::NodeHandle * n){
	Msg_20FF_pub = n->advertise<hg_nav_node::Msg_20FF>(MSG_20FF_PATH, 5);
	Msg_20FF_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_20FF_PATH);
	return;
}

void stop_20FF(void){
	Msg_20FF_pub.shutdown();
	Msg_20FF_pub_initialized = false;
	ROS_INFO("0x20FF stopped");
	return;
}

// Msg_20FF to Topic
void convert(Msg_20FF messageIn, hg_nav_node::Msg_20FF * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Ack = messageIn.Ack;
	messageOut->InputMessageID = messageIn.InputMessageID;
	messageOut->NoOfValidMessagesSinceLast = messageIn.NoOfValidMessagesSinceLast;
	messageOut->NoOfValidMessagesSincePowerUp = messageIn.NoOfValidMessagesSincePowerUp;
	messageOut->MessageTimeOfReception = messageIn.MessageTimeOfReception;
}

// Topic to Msg_20FF
void convert(hg_nav_node::Msg_20FF messageIn, Msg_20FF * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Ack = messageIn.Ack;
	messageOut->InputMessageID = messageIn.InputMessageID;
	messageOut->NoOfValidMessagesSinceLast = messageIn.NoOfValidMessagesSinceLast;
	messageOut->NoOfValidMessagesSincePowerUp = messageIn.NoOfValidMessagesSincePowerUp;
	messageOut->MessageTimeOfReception = messageIn.MessageTimeOfReception;
}

void Msg_20FF_pub_callback(uint8_t * buffer)
{
	Msg_20FF Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x20FF deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_20FF);
	ROS_DEBUG("Message 0x20FF Received");

	// Initialize Publisher if not initialized yet
	if (Msg_20FF_pub_initialized == false){
		init_20FF(getRosHandle());}
	// Publish the message
	Msg_20FF_pub.publish(msgStruct_20FF);
	return;
}
