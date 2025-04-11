#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4204.h>
hg_nav_node::Msg_4204 msgStruct_4204;

ros::Subscriber Msg_4204_sub;
void init_4204(ros::NodeHandle * n){
	Msg_4204_sub = n->subscribe(MSG_4204_PATH, 5, Msg_4204_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4204_PATH);
	return;
}

void stop_4204(void){
	Msg_4204_sub.shutdown();
	ROS_INFO("0x4204 stopped");
	return;
}

// Msg_4204 to Topic
void convert(Msg_4204 messageIn, hg_nav_node::Msg_4204 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RF1AntennaX = messageIn.RF1AntennaX;
	messageOut->RF1AntennaY = messageIn.RF1AntennaY;
	messageOut->RF1AntennaZ = messageIn.RF1AntennaZ;
	messageOut->RF2AntennaX = messageIn.RF2AntennaX;
	messageOut->RF2AntennaY = messageIn.RF2AntennaY;
	messageOut->RF2AntennaZ = messageIn.RF2AntennaZ;
	messageOut->RF1AntennaSTDV = messageIn.RF1AntennaSTDV;
	messageOut->RF2AntennaSTDV = messageIn.RF2AntennaSTDV;
	messageOut->ChangeRF1LeverArm = messageIn.ChangeRF1LeverArm;
	messageOut->ChangeRF2LeverArm = messageIn.ChangeRF2LeverArm;
	messageOut->ChangeRF1STDV = messageIn.ChangeRF1STDV;
	messageOut->ChangeRF2STDV = messageIn.ChangeRF2STDV;
}

// Topic to Msg_4204
void convert(hg_nav_node::Msg_4204 messageIn, Msg_4204 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RF1AntennaX = messageIn.RF1AntennaX;
	messageOut->RF1AntennaY = messageIn.RF1AntennaY;
	messageOut->RF1AntennaZ = messageIn.RF1AntennaZ;
	messageOut->RF2AntennaX = messageIn.RF2AntennaX;
	messageOut->RF2AntennaY = messageIn.RF2AntennaY;
	messageOut->RF2AntennaZ = messageIn.RF2AntennaZ;
	messageOut->RF1AntennaSTDV = messageIn.RF1AntennaSTDV;
	messageOut->RF2AntennaSTDV = messageIn.RF2AntennaSTDV;
	messageOut->ChangeRF1LeverArm = messageIn.ChangeRF1LeverArm;
	messageOut->ChangeRF2LeverArm = messageIn.ChangeRF2LeverArm;
	messageOut->ChangeRF1STDV = messageIn.ChangeRF1STDV;
	messageOut->ChangeRF2STDV = messageIn.ChangeRF2STDV;
}

void Msg_4204_sub_callback(const hg_nav_node::Msg_4204::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4204 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4204 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4204 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
