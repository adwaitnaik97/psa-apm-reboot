#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4110.h>
#include <hg_nav_node/odo_body_vel_status_t.h>
hg_nav_node::Msg_4110 msgStruct_4110;

ros::Subscriber Msg_4110_sub;
void init_4110(ros::NodeHandle * n){
	Msg_4110_sub = n->subscribe(MSG_4110_PATH, 5, Msg_4110_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4110_PATH);
	return;
}

void stop_4110(void){
	Msg_4110_sub.shutdown();
	ROS_INFO("0x4110 stopped");
	return;
}

// Msg_4110 to Topic
void convert(Msg_4110 messageIn, hg_nav_node::Msg_4110 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->TimeOfValidityAdjustment = messageIn.TimeOfValidityAdjustment;
	messageOut->GPS_TOV = messageIn.GPS_TOV;
	messageOut->Body_Velocity_X = messageIn.Body_Velocity_X;
	messageOut->Body_Velocity_Y = messageIn.Body_Velocity_Y;
	messageOut->Body_Velocity_Z = messageIn.Body_Velocity_Z;
	messageOut->Odo_Pulses = messageIn.Odo_Pulses;
	messageOut->Distance_per_Pulse = messageIn.Distance_per_Pulse;

	messageOut->Odo_Body_Vel_Status.Velocity_Valid = messageIn.Odo_Body_Vel_Status.Velocity_Valid;
	messageOut->Odo_Body_Vel_Status.Odo_Pulse_Valid = messageIn.Odo_Body_Vel_Status.Odo_Pulse_Valid;
	messageOut->Odo_Body_Vel_Status.TOV_Mode = messageIn.Odo_Body_Vel_Status.TOV_Mode;
	messageOut->Odo_Body_Vel_Status.Vel_Sending_Unit_Status = messageIn.Odo_Body_Vel_Status.Vel_Sending_Unit_Status;
	messageOut->Odo_Body_Vel_Status.Zupt_Requested = messageIn.Odo_Body_Vel_Status.Zupt_Requested;
	messageOut->Message_Counter = messageIn.Message_Counter;
	messageOut->X_Y_Velocity_STDV = messageIn.X_Y_Velocity_STDV;
	messageOut->Z_Velocity_STDV = messageIn.Z_Velocity_STDV;
}

// Topic to Msg_4110
void convert(hg_nav_node::Msg_4110 messageIn, Msg_4110 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->TimeOfValidityAdjustment = messageIn.TimeOfValidityAdjustment;
	messageOut->GPS_TOV = messageIn.GPS_TOV;
	messageOut->Body_Velocity_X = messageIn.Body_Velocity_X;
	messageOut->Body_Velocity_Y = messageIn.Body_Velocity_Y;
	messageOut->Body_Velocity_Z = messageIn.Body_Velocity_Z;
	messageOut->Odo_Pulses = messageIn.Odo_Pulses;
	messageOut->Distance_per_Pulse = messageIn.Distance_per_Pulse;

	messageOut->Odo_Body_Vel_Status.Velocity_Valid = messageIn.Odo_Body_Vel_Status.Velocity_Valid;
	messageOut->Odo_Body_Vel_Status.Odo_Pulse_Valid = messageIn.Odo_Body_Vel_Status.Odo_Pulse_Valid;
	messageOut->Odo_Body_Vel_Status.TOV_Mode = messageIn.Odo_Body_Vel_Status.TOV_Mode;
	messageOut->Odo_Body_Vel_Status.Vel_Sending_Unit_Status = messageIn.Odo_Body_Vel_Status.Vel_Sending_Unit_Status;
	messageOut->Odo_Body_Vel_Status.Zupt_Requested = messageIn.Odo_Body_Vel_Status.Zupt_Requested;
	messageOut->Message_Counter = messageIn.Message_Counter;
	messageOut->X_Y_Velocity_STDV = messageIn.X_Y_Velocity_STDV;
	messageOut->Z_Velocity_STDV = messageIn.Z_Velocity_STDV;
}

void Msg_4110_sub_callback(const hg_nav_node::Msg_4110::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4110 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4110 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4110 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
