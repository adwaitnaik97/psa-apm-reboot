#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1101.h>
hg_nav_node::Msg_1101 msgStruct_1101;

ros::Subscriber Msg_1101_sub;
void init_1101(ros::NodeHandle * n){
	Msg_1101_sub = n->subscribe(MSG_1101_PATH, 5, Msg_1101_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1101_PATH);
	return;
}

void stop_1101(void){
	Msg_1101_sub.shutdown();
	ROS_INFO("0x1101 stopped");
	return;
}

// Msg_1101 to Topic
void convert(Msg_1101 messageIn, hg_nav_node::Msg_1101 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->BarometricAltitudeTov = messageIn.BarometricAltitudeTov;
	messageOut->BarometricAltitudeValidity = messageIn.BarometricAltitudeValidity;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->BarometricAltitudeMSLGeoid = messageIn.BarometricAltitudeMSLGeoid;
}

// Topic to Msg_1101
void convert(hg_nav_node::Msg_1101 messageIn, Msg_1101 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->BarometricAltitudeTov = messageIn.BarometricAltitudeTov;
	messageOut->BarometricAltitudeValidity = messageIn.BarometricAltitudeValidity;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->BarometricAltitudeMSLGeoid = messageIn.BarometricAltitudeMSLGeoid;
}

void Msg_1101_sub_callback(const hg_nav_node::Msg_1101::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1101 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1101 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1101 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
