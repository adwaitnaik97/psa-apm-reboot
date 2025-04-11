#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2426.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2426 msgStruct_2426;

bool Msg_2426_pub_initialized = false;

ros::Publisher Msg_2426_pub;
void init_2426(ros::NodeHandle * n){
	Msg_2426_pub = n->advertise<hg_nav_node::Msg_2426>(MSG_2426_PATH, 5);
	Msg_2426_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2426_PATH);
	return;
}

void stop_2426(void){
	Msg_2426_pub.shutdown();
	Msg_2426_pub_initialized = false;
	ROS_INFO("0x2426 stopped");
	return;
}

// Msg_2426 to Topic
void convert(Msg_2426 messageIn, hg_nav_node::Msg_2426 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->BaroBiasErrEst = messageIn.BaroBiasErrEst;
	messageOut->BaroScaleFactorErrEst = messageIn.BaroScaleFactorErrEst;
	messageOut->BaroBiasSTDV = messageIn.BaroBiasSTDV;
	messageOut->BaroScaleFactorSTDV = messageIn.BaroScaleFactorSTDV;
	messageOut->BaroMeasResid = messageIn.BaroMeasResid;
}

// Topic to Msg_2426
void convert(hg_nav_node::Msg_2426 messageIn, Msg_2426 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->BaroBiasErrEst = messageIn.BaroBiasErrEst;
	messageOut->BaroScaleFactorErrEst = messageIn.BaroScaleFactorErrEst;
	messageOut->BaroBiasSTDV = messageIn.BaroBiasSTDV;
	messageOut->BaroScaleFactorSTDV = messageIn.BaroScaleFactorSTDV;
	messageOut->BaroMeasResid = messageIn.BaroMeasResid;
}

void Msg_2426_pub_callback(uint8_t * buffer)
{
	Msg_2426 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2426 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2426);
	ROS_DEBUG("Message 0x2426 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2426_pub_initialized == false){
		init_2426(getRosHandle());}
	// Publish the message
	Msg_2426_pub.publish(msgStruct_2426);
	return;
}
