#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4438.h>
hg_nav_node::Msg_4438 msgStruct_4438;

ros::Subscriber Msg_4438_sub;
void init_4438(ros::NodeHandle * n){
	Msg_4438_sub = n->subscribe(MSG_4438_PATH, 5, Msg_4438_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4438_PATH);
	return;
}

void stop_4438(void){
	Msg_4438_sub.shutdown();
	ROS_INFO("0x4438 stopped");
	return;
}

// Msg_4438 to Topic
void convert(Msg_4438 messageIn, hg_nav_node::Msg_4438 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->QdecEnable = messageIn.QdecEnable;
	messageOut->QdecDistancePerPulse = messageIn.QdecDistancePerPulse;
	messageOut->OdometerLeverArmX = messageIn.OdometerLeverArmX;
	messageOut->OdometerLeverArmY = messageIn.OdometerLeverArmY;
	messageOut->OdometerLeverArmZ = messageIn.OdometerLeverArmZ;
	messageOut->OdometerMeasurementNoise = messageIn.OdometerMeasurementNoise;
	messageOut->OdometerTheshold = messageIn.OdometerTheshold;
	messageOut->OdometerInitialScaleFactorUncertainty = messageIn.OdometerInitialScaleFactorUncertainty;
	messageOut->OdometerScaleFactorNoise = messageIn.OdometerScaleFactorNoise;
	messageOut->OdometerYawBoresightStdv = messageIn.OdometerYawBoresightStdv;
	messageOut->OdometerYawBoresightProcessNoise = messageIn.OdometerYawBoresightProcessNoise;
	messageOut->OdometerInitialPitchBoresightUncertainty = messageIn.OdometerInitialPitchBoresightUncertainty;
	messageOut->OdometerInitialPitchBoresightProcessNoise = messageIn.OdometerInitialPitchBoresightProcessNoise;
	messageOut->ChangeOdometerLeverArm = messageIn.ChangeOdometerLeverArm;
}

// Topic to Msg_4438
void convert(hg_nav_node::Msg_4438 messageIn, Msg_4438 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->QdecEnable = messageIn.QdecEnable;
	messageOut->QdecDistancePerPulse = messageIn.QdecDistancePerPulse;
	messageOut->OdometerLeverArmX = messageIn.OdometerLeverArmX;
	messageOut->OdometerLeverArmY = messageIn.OdometerLeverArmY;
	messageOut->OdometerLeverArmZ = messageIn.OdometerLeverArmZ;
	messageOut->OdometerMeasurementNoise = messageIn.OdometerMeasurementNoise;
	messageOut->OdometerTheshold = messageIn.OdometerTheshold;
	messageOut->OdometerInitialScaleFactorUncertainty = messageIn.OdometerInitialScaleFactorUncertainty;
	messageOut->OdometerScaleFactorNoise = messageIn.OdometerScaleFactorNoise;
	messageOut->OdometerYawBoresightStdv = messageIn.OdometerYawBoresightStdv;
	messageOut->OdometerYawBoresightProcessNoise = messageIn.OdometerYawBoresightProcessNoise;
	messageOut->OdometerInitialPitchBoresightUncertainty = messageIn.OdometerInitialPitchBoresightUncertainty;
	messageOut->OdometerInitialPitchBoresightProcessNoise = messageIn.OdometerInitialPitchBoresightProcessNoise;
	messageOut->ChangeOdometerLeverArm = messageIn.ChangeOdometerLeverArm;
}

void Msg_4438_sub_callback(const hg_nav_node::Msg_4438::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4438 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4438 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4438 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
