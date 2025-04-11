#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4201.h>
#include <hg_nav_node/event_in_mark_t.h>
hg_nav_node::Msg_4201 msgStruct_4201;

ros::Subscriber Msg_4201_sub;
void init_4201(ros::NodeHandle * n){
	Msg_4201_sub = n->subscribe(MSG_4201_PATH, 5, Msg_4201_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4201_PATH);
	return;
}

void stop_4201(void){
	Msg_4201_sub.shutdown();
	ROS_INFO("0x4201 stopped");
	return;
}

// Msg_4201 to Topic
void convert(Msg_4201 messageIn, hg_nav_node::Msg_4201 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MarkPort.value = static_cast<uint8_t>(messageIn.MarkPort);
	messageOut->Enable_6201_Time = messageIn.Enable_6201_Time;
	messageOut->Enable_6202_Position = messageIn.Enable_6202_Position;
	messageOut->Enable_6203_Velocity = messageIn.Enable_6203_Velocity;
	messageOut->Enable_6204_Attitude = messageIn.Enable_6204_Attitude;
	messageOut->Enable_6205_Full_State = messageIn.Enable_6205_Full_State;
	messageOut->ChangeEventInSetUp = messageIn.ChangeEventInSetUp;
	messageOut->Polarity = messageIn.Polarity;
	messageOut->time_offset = messageIn.time_offset;
	messageOut->time_guard = messageIn.time_guard;
}

// Topic to Msg_4201
void convert(hg_nav_node::Msg_4201 messageIn, Msg_4201 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MarkPort = static_cast<event_in_mark_t>(messageIn.MarkPort.value);
	messageOut->Enable_6201_Time = messageIn.Enable_6201_Time;
	messageOut->Enable_6202_Position = messageIn.Enable_6202_Position;
	messageOut->Enable_6203_Velocity = messageIn.Enable_6203_Velocity;
	messageOut->Enable_6204_Attitude = messageIn.Enable_6204_Attitude;
	messageOut->Enable_6205_Full_State = messageIn.Enable_6205_Full_State;
	messageOut->ChangeEventInSetUp = messageIn.ChangeEventInSetUp;
	messageOut->Polarity = messageIn.Polarity;
	messageOut->time_offset = messageIn.time_offset;
	messageOut->time_guard = messageIn.time_guard;
}

void Msg_4201_sub_callback(const hg_nav_node::Msg_4201::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4201 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4201 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4201 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
