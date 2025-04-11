#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4404.h>
hg_nav_node::Msg_4404 msgStruct_4404;

ros::Subscriber Msg_4404_sub;
void init_4404(ros::NodeHandle * n){
	Msg_4404_sub = n->subscribe(MSG_4404_PATH, 5, Msg_4404_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4404_PATH);
	return;
}

void stop_4404(void){
	Msg_4404_sub.shutdown();
	ROS_INFO("0x4404 stopped");
	return;
}

// Msg_4404 to Topic
void convert(Msg_4404 messageIn, hg_nav_node::Msg_4404 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->CasetoVehicleX = messageIn.CasetoVehicleX;
	messageOut->CasetoVehicleY = messageIn.CasetoVehicleY;
	messageOut->CasetoVehicleZ = messageIn.CasetoVehicleZ;
	messageOut->CasetoVehicleRoll = messageIn.CasetoVehicleRoll;
	messageOut->CasetoVehiclePitch = messageIn.CasetoVehiclePitch;
	messageOut->CasetoVehicleYaw = messageIn.CasetoVehicleYaw;
	messageOut->ChangeVehicleFrameAttitude = messageIn.ChangeVehicleFrameAttitude;
	messageOut->ChangeVehicleFrameOffset = messageIn.ChangeVehicleFrameOffset;
}

// Topic to Msg_4404
void convert(hg_nav_node::Msg_4404 messageIn, Msg_4404 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->CasetoVehicleX = messageIn.CasetoVehicleX;
	messageOut->CasetoVehicleY = messageIn.CasetoVehicleY;
	messageOut->CasetoVehicleZ = messageIn.CasetoVehicleZ;
	messageOut->CasetoVehicleRoll = messageIn.CasetoVehicleRoll;
	messageOut->CasetoVehiclePitch = messageIn.CasetoVehiclePitch;
	messageOut->CasetoVehicleYaw = messageIn.CasetoVehicleYaw;
	messageOut->ChangeVehicleFrameAttitude = messageIn.ChangeVehicleFrameAttitude;
	messageOut->ChangeVehicleFrameOffset = messageIn.ChangeVehicleFrameOffset;
}

void Msg_4404_sub_callback(const hg_nav_node::Msg_4404::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4404 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4404 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4404 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
