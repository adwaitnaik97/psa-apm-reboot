#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4401.h>
#include <hg_nav_node/coordinate_frame_t.h>
hg_nav_node::Msg_4401 msgStruct_4401;

ros::Subscriber Msg_4401_sub;
void init_4401(ros::NodeHandle * n){
	Msg_4401_sub = n->subscribe(MSG_4401_PATH, 5, Msg_4401_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4401_PATH);
	return;
}

void stop_4401(void){
	Msg_4401_sub.shutdown();
	ROS_INFO("0x4401 stopped");
	return;
}

// Msg_4401 to Topic
void convert(Msg_4401 messageIn, hg_nav_node::Msg_4401 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestACKNAKReply = messageIn.RequestACKNAKReply;
	messageOut->TOV_Mode = messageIn.TOV_Mode;
	messageOut->input_coordinate_frame.value = static_cast<uint8_t>(messageIn.input_coordinate_frame);
	messageOut->solutionTov = messageIn.solutionTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->Altitude = messageIn.Altitude;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->AttitudeValidity = messageIn.AttitudeValidity;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->TrueHeading = messageIn.TrueHeading;
	messageOut->PositionStdvNorth = messageIn.PositionStdvNorth;
	messageOut->PositionStdvEast = messageIn.PositionStdvEast;
	messageOut->PositionStdvDown = messageIn.PositionStdvDown;
	messageOut->VelocityStdvNorth = messageIn.VelocityStdvNorth;
	messageOut->VelocityStdvEast = messageIn.VelocityStdvEast;
	messageOut->VelocityStdvDown = messageIn.VelocityStdvDown;
	messageOut->AttitudeStdvRoll = messageIn.AttitudeStdvRoll;
	messageOut->AttitudeStdvPitch = messageIn.AttitudeStdvPitch;
	messageOut->AttitudeStdvTrueHeading = messageIn.AttitudeStdvTrueHeading;
}

// Topic to Msg_4401
void convert(hg_nav_node::Msg_4401 messageIn, Msg_4401 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestACKNAKReply = messageIn.RequestACKNAKReply;
	messageOut->TOV_Mode = messageIn.TOV_Mode;
	messageOut->input_coordinate_frame = static_cast<coordinate_frame_t>(messageIn.input_coordinate_frame.value);
	messageOut->solutionTov = messageIn.solutionTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->Altitude = messageIn.Altitude;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->AttitudeValidity = messageIn.AttitudeValidity;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->TrueHeading = messageIn.TrueHeading;
	messageOut->PositionStdvNorth = messageIn.PositionStdvNorth;
	messageOut->PositionStdvEast = messageIn.PositionStdvEast;
	messageOut->PositionStdvDown = messageIn.PositionStdvDown;
	messageOut->VelocityStdvNorth = messageIn.VelocityStdvNorth;
	messageOut->VelocityStdvEast = messageIn.VelocityStdvEast;
	messageOut->VelocityStdvDown = messageIn.VelocityStdvDown;
	messageOut->AttitudeStdvRoll = messageIn.AttitudeStdvRoll;
	messageOut->AttitudeStdvPitch = messageIn.AttitudeStdvPitch;
	messageOut->AttitudeStdvTrueHeading = messageIn.AttitudeStdvTrueHeading;
}

void Msg_4401_sub_callback(const hg_nav_node::Msg_4401::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4401 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4401 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4401 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
