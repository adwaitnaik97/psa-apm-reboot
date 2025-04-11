#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1105.h>
hg_nav_node::Msg_1105 msgStruct_1105;

ros::Subscriber Msg_1105_sub;
void init_1105(ros::NodeHandle * n){
	Msg_1105_sub = n->subscribe(MSG_1105_PATH, 5, Msg_1105_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1105_PATH);
	return;
}

void stop_1105(void){
	Msg_1105_sub.shutdown();
	ROS_INFO("0x1105 stopped");
	return;
}

// Msg_1105 to Topic
void convert(Msg_1105 messageIn, hg_nav_node::Msg_1105 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MagneticHeadingTov = messageIn.MagneticHeadingTov;
	messageOut->MagneticHeadingValidity = messageIn.MagneticHeadingValidity;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->MagneticVariationValidity = messageIn.MagneticVariationValidity;
	messageOut->MagneticHeading = messageIn.MagneticHeading;
	messageOut->MagneticVariation = messageIn.MagneticVariation;
}

// Topic to Msg_1105
void convert(hg_nav_node::Msg_1105 messageIn, Msg_1105 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MagneticHeadingTov = messageIn.MagneticHeadingTov;
	messageOut->MagneticHeadingValidity = messageIn.MagneticHeadingValidity;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->MagneticVariationValidity = messageIn.MagneticVariationValidity;
	messageOut->MagneticHeading = messageIn.MagneticHeading;
	messageOut->MagneticVariation = messageIn.MagneticVariation;
}

void Msg_1105_sub_callback(const hg_nav_node::Msg_1105::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1105 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1105 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1105 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
