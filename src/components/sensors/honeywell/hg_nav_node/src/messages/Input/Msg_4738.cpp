#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4738.h>
hg_nav_node::Msg_4738 msgStruct_4738;

ros::Subscriber Msg_4738_sub;
void init_4738(ros::NodeHandle * n){
	Msg_4738_sub = n->subscribe(MSG_4738_PATH, 5, Msg_4738_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4738_PATH);
	return;
}

void stop_4738(void){
	Msg_4738_sub.shutdown();
	ROS_INFO("0x4738 stopped");
	return;
}

// Msg_4738 to Topic
void convert(Msg_4738 messageIn, hg_nav_node::Msg_4738 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->measNoise = messageIn.measNoise;
	messageOut->measRejectThreshold = messageIn.measRejectThreshold;
	messageOut->sfUncertainty = messageIn.sfUncertainty;
	messageOut->sfTimeConst = messageIn.sfTimeConst;
	messageOut->misalignInitUncertainty = messageIn.misalignInitUncertainty;
	messageOut->misalignTimeConst = messageIn.misalignTimeConst;
	messageOut->pitchBoresightInitUncertainty = messageIn.pitchBoresightInitUncertainty;
	messageOut->pitchBoresightStateNoise = messageIn.pitchBoresightStateNoise;
	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam0[index] = messageIn.leverarmBeam0[index];
	}

	messageOut->psiRotationBeam0 = messageIn.psiRotationBeam0;
	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam1[index] = messageIn.leverarmBeam1[index];
	}

	messageOut->psiRotationBeam1 = messageIn.psiRotationBeam1;
	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam2[index] = messageIn.leverarmBeam2[index];
	}

	messageOut->psiRotationBeam2 = messageIn.psiRotationBeam2;
	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam3[index] = messageIn.leverarmBeam3[index];
	}

	messageOut->psiRotationBeam3 = messageIn.psiRotationBeam3;
	messageOut->psiThtRotation = messageIn.psiThtRotation;
	messageOut->psiOffset = messageIn.psiOffset;
	messageOut->saveCfgToFlash = messageIn.saveCfgToFlash;
}

// Topic to Msg_4738
void convert(hg_nav_node::Msg_4738 messageIn, Msg_4738 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->measNoise = messageIn.measNoise;
	messageOut->measRejectThreshold = messageIn.measRejectThreshold;
	messageOut->sfUncertainty = messageIn.sfUncertainty;
	messageOut->sfTimeConst = messageIn.sfTimeConst;
	messageOut->misalignInitUncertainty = messageIn.misalignInitUncertainty;
	messageOut->misalignTimeConst = messageIn.misalignTimeConst;
	messageOut->pitchBoresightInitUncertainty = messageIn.pitchBoresightInitUncertainty;
	messageOut->pitchBoresightStateNoise = messageIn.pitchBoresightStateNoise;	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam0[index] = messageIn.leverarmBeam0[index];
	}

	messageOut->psiRotationBeam0 = messageIn.psiRotationBeam0;	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam1[index] = messageIn.leverarmBeam1[index];
	}

	messageOut->psiRotationBeam1 = messageIn.psiRotationBeam1;	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam2[index] = messageIn.leverarmBeam2[index];
	}

	messageOut->psiRotationBeam2 = messageIn.psiRotationBeam2;	for (unsigned int index = 0; index < 3; index++)
	{
		messageOut->leverarmBeam3[index] = messageIn.leverarmBeam3[index];
	}

	messageOut->psiRotationBeam3 = messageIn.psiRotationBeam3;
	messageOut->psiThtRotation = messageIn.psiThtRotation;
	messageOut->psiOffset = messageIn.psiOffset;
	messageOut->saveCfgToFlash = messageIn.saveCfgToFlash;
}

void Msg_4738_sub_callback(const hg_nav_node::Msg_4738::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4738 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4738 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4738 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
