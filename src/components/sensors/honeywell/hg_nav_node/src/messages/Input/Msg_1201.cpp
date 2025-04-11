#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1201.h>
hg_nav_node::Msg_1201 msgStruct_1201;

ros::Subscriber Msg_1201_sub;
void init_1201(ros::NodeHandle * n){
	Msg_1201_sub = n->subscribe(MSG_1201_PATH, 5, Msg_1201_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1201_PATH);
	return;
}

void stop_1201(void){
	Msg_1201_sub.shutdown();
	ROS_INFO("0x1201 stopped");
	return;
}

// Msg_1201 to Topic
void convert(Msg_1201 messageIn, hg_nav_node::Msg_1201 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTovValid = messageIn.gpsTovValid;
	messageOut->gpsToUTCofsetValid = messageIn.gpsToUTCofsetValid;
	messageOut->gps_time_to_utc_offset = messageIn.gps_time_to_utc_offset;
	messageOut->PPSgpsTov = messageIn.PPSgpsTov;
	messageOut->gps_week = messageIn.gps_week;
}

// Topic to Msg_1201
void convert(hg_nav_node::Msg_1201 messageIn, Msg_1201 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTovValid = messageIn.gpsTovValid;
	messageOut->gpsToUTCofsetValid = messageIn.gpsToUTCofsetValid;
	messageOut->gps_time_to_utc_offset = messageIn.gps_time_to_utc_offset;
	messageOut->PPSgpsTov = messageIn.PPSgpsTov;
	messageOut->gps_week = messageIn.gps_week;
}

void Msg_1201_sub_callback(const hg_nav_node::Msg_1201::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1201 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1201 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1201 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
