#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4651.h>
#include <hg_nav_node/check_enable_t.h>
hg_nav_node::Msg_4651 msgStruct_4651;

ros::Subscriber Msg_4651_sub;
void init_4651(ros::NodeHandle * n){
	Msg_4651_sub = n->subscribe(MSG_4651_PATH, 5, Msg_4651_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4651_PATH);
	return;
}

void stop_4651(void){
	Msg_4651_sub.shutdown();
	ROS_INFO("0x4651 stopped");
	return;
}

// Msg_4651 to Topic
void convert(Msg_4651 messageIn, hg_nav_node::Msg_4651 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->WarmupTime = messageIn.WarmupTime;

	messageOut->ThresholdCheckEnable.position = messageIn.ThresholdCheckEnable.position;
	messageOut->ThresholdCheckEnable.velocity = messageIn.ThresholdCheckEnable.velocity;
	messageOut->ThresholdCheckEnable.attitude = messageIn.ThresholdCheckEnable.attitude;
	messageOut->ThresholdCheckEnable.position = messageIn.ThresholdCheckEnable.position;
	messageOut->ThresholdCheckEnable.velocity = messageIn.ThresholdCheckEnable.velocity;
	messageOut->ThresholdCheckEnable.attitude = messageIn.ThresholdCheckEnable.attitude;
	messageOut->PosXErrorThreshold = messageIn.PosXErrorThreshold;
	messageOut->PosYErrorThreshold = messageIn.PosYErrorThreshold;
	messageOut->PosZErrorThreshold = messageIn.PosZErrorThreshold;
	messageOut->VelXErrorThreshold = messageIn.VelXErrorThreshold;
	messageOut->VelYErrorThreshold = messageIn.VelYErrorThreshold;
	messageOut->VelZErrorThreshold = messageIn.VelZErrorThreshold;
	messageOut->AttXErrorThreshold = messageIn.AttXErrorThreshold;
	messageOut->AttYErrorThreshold = messageIn.AttYErrorThreshold;
	messageOut->AttZErrorThreshold = messageIn.AttZErrorThreshold;
	messageOut->NavAxesSelector = messageIn.NavAxesSelector;
}

// Topic to Msg_4651
void convert(hg_nav_node::Msg_4651 messageIn, Msg_4651 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->WarmupTime = messageIn.WarmupTime;

	messageOut->ThresholdCheckEnable.position = messageIn.ThresholdCheckEnable.position;
	messageOut->ThresholdCheckEnable.velocity = messageIn.ThresholdCheckEnable.velocity;
	messageOut->ThresholdCheckEnable.attitude = messageIn.ThresholdCheckEnable.attitude;
	messageOut->ThresholdCheckEnable.position = messageIn.ThresholdCheckEnable.position;
	messageOut->ThresholdCheckEnable.velocity = messageIn.ThresholdCheckEnable.velocity;
	messageOut->ThresholdCheckEnable.attitude = messageIn.ThresholdCheckEnable.attitude;
	messageOut->PosXErrorThreshold = messageIn.PosXErrorThreshold;
	messageOut->PosYErrorThreshold = messageIn.PosYErrorThreshold;
	messageOut->PosZErrorThreshold = messageIn.PosZErrorThreshold;
	messageOut->VelXErrorThreshold = messageIn.VelXErrorThreshold;
	messageOut->VelYErrorThreshold = messageIn.VelYErrorThreshold;
	messageOut->VelZErrorThreshold = messageIn.VelZErrorThreshold;
	messageOut->AttXErrorThreshold = messageIn.AttXErrorThreshold;
	messageOut->AttYErrorThreshold = messageIn.AttYErrorThreshold;
	messageOut->AttZErrorThreshold = messageIn.AttZErrorThreshold;
	messageOut->NavAxesSelector = messageIn.NavAxesSelector;
}

void Msg_4651_sub_callback(const hg_nav_node::Msg_4651::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4651 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4651 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4651 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
