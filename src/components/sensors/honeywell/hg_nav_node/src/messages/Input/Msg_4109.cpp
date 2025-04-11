#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4109.h>
hg_nav_node::Msg_4109 msgStruct_4109;

ros::Subscriber Msg_4109_sub;
void init_4109(ros::NodeHandle * n){
	Msg_4109_sub = n->subscribe(MSG_4109_PATH, 5, Msg_4109_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4109_PATH);
	return;
}

void stop_4109(void){
	Msg_4109_sub.shutdown();
	ROS_INFO("0x4109 stopped");
	return;
}

// Msg_4109 to Topic
void convert(Msg_4109 messageIn, hg_nav_node::Msg_4109 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestAckNak = messageIn.RequestAckNak;
	messageOut->RollValid = messageIn.RollValid;
	messageOut->PitchValid = messageIn.PitchValid;
	messageOut->HeadingValid = messageIn.HeadingValid;
	messageOut->TovMode = messageIn.TovMode;
	messageOut->MessageTov = messageIn.MessageTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->TrueHdg = messageIn.TrueHdg;
	messageOut->RollStdv = messageIn.RollStdv;
	messageOut->PitchStdv = messageIn.PitchStdv;
	messageOut->TrueHdgStdv = messageIn.TrueHdgStdv;
	messageOut->Baseline = messageIn.Baseline;
}

// Topic to Msg_4109
void convert(hg_nav_node::Msg_4109 messageIn, Msg_4109 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestAckNak = messageIn.RequestAckNak;
	messageOut->RollValid = messageIn.RollValid;
	messageOut->PitchValid = messageIn.PitchValid;
	messageOut->HeadingValid = messageIn.HeadingValid;
	messageOut->TovMode = messageIn.TovMode;
	messageOut->MessageTov = messageIn.MessageTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->TrueHdg = messageIn.TrueHdg;
	messageOut->RollStdv = messageIn.RollStdv;
	messageOut->PitchStdv = messageIn.PitchStdv;
	messageOut->TrueHdgStdv = messageIn.TrueHdgStdv;
	messageOut->Baseline = messageIn.Baseline;
}

void Msg_4109_sub_callback(const hg_nav_node::Msg_4109::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4109 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4109 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4109 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
