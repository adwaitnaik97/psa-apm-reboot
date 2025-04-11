#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4202.h>
#include <hg_nav_node/event_out_mark_t.h>
hg_nav_node::Msg_4202 msgStruct_4202;

ros::Subscriber Msg_4202_sub;
void init_4202(ros::NodeHandle * n){
	Msg_4202_sub = n->subscribe(MSG_4202_PATH, 5, Msg_4202_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4202_PATH);
	return;
}

void stop_4202(void){
	Msg_4202_sub.shutdown();
	ROS_INFO("0x4202 stopped");
	return;
}

// Msg_4202 to Topic
void convert(Msg_4202 messageIn, hg_nav_node::Msg_4202 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MarkPort.value = static_cast<uint8_t>(messageIn.MarkPort);
	messageOut->Polarity = messageIn.Polarity;
	messageOut->Enable_6211_Time = messageIn.Enable_6211_Time;
	messageOut->ChangeEventOutSetUp = messageIn.ChangeEventOutSetUp;
	messageOut->Trigger = messageIn.Trigger;
	messageOut->Trigger_Value = messageIn.Trigger_Value;
	messageOut->time_offset = messageIn.time_offset;
	messageOut->pulse_width = messageIn.pulse_width;
}

// Topic to Msg_4202
void convert(hg_nav_node::Msg_4202 messageIn, Msg_4202 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MarkPort = static_cast<event_out_mark_t>(messageIn.MarkPort.value);
	messageOut->Polarity = messageIn.Polarity;
	messageOut->Enable_6211_Time = messageIn.Enable_6211_Time;
	messageOut->ChangeEventOutSetUp = messageIn.ChangeEventOutSetUp;
	messageOut->Trigger = messageIn.Trigger;
	messageOut->Trigger_Value = messageIn.Trigger_Value;
	messageOut->time_offset = messageIn.time_offset;
	messageOut->pulse_width = messageIn.pulse_width;
}

void Msg_4202_sub_callback(const hg_nav_node::Msg_4202::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4202 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4202 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4202 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
