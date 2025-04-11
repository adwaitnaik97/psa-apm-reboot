#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2429.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2429 msgStruct_2429;

bool Msg_2429_pub_initialized = false;

ros::Publisher Msg_2429_pub;
void init_2429(ros::NodeHandle * n){
	Msg_2429_pub = n->advertise<hg_nav_node::Msg_2429>(MSG_2429_PATH, 5);
	Msg_2429_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2429_PATH);
	return;
}

void stop_2429(void){
	Msg_2429_pub.shutdown();
	Msg_2429_pub_initialized = false;
	ROS_INFO("0x2429 stopped");
	return;
}

// Msg_2429 to Topic
void convert(Msg_2429 messageIn, hg_nav_node::Msg_2429 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->TA_PositionX_MeasResid = messageIn.TA_PositionX_MeasResid;
	messageOut->TA_PositionY_MeasResid = messageIn.TA_PositionY_MeasResid;
	messageOut->TA_PositionZ_MeasResid = messageIn.TA_PositionZ_MeasResid;
	messageOut->TA_VelocityX_MeasResid = messageIn.TA_VelocityX_MeasResid;
	messageOut->TA_VelocityY_MeasResid = messageIn.TA_VelocityY_MeasResid;
	messageOut->TA_VelocityZ_MeasResid = messageIn.TA_VelocityZ_MeasResid;
	messageOut->TA_AttitudeX_MeasResid = messageIn.TA_AttitudeX_MeasResid;
	messageOut->TA_AttitudeY_MeasResid = messageIn.TA_AttitudeY_MeasResid;
	messageOut->TA_AttitudeZ_MeasResid = messageIn.TA_AttitudeZ_MeasResid;
	messageOut->TA_AttitudeCosZ_MeasResid = messageIn.TA_AttitudeCosZ_MeasResid;
}

// Topic to Msg_2429
void convert(hg_nav_node::Msg_2429 messageIn, Msg_2429 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->TA_PositionX_MeasResid = messageIn.TA_PositionX_MeasResid;
	messageOut->TA_PositionY_MeasResid = messageIn.TA_PositionY_MeasResid;
	messageOut->TA_PositionZ_MeasResid = messageIn.TA_PositionZ_MeasResid;
	messageOut->TA_VelocityX_MeasResid = messageIn.TA_VelocityX_MeasResid;
	messageOut->TA_VelocityY_MeasResid = messageIn.TA_VelocityY_MeasResid;
	messageOut->TA_VelocityZ_MeasResid = messageIn.TA_VelocityZ_MeasResid;
	messageOut->TA_AttitudeX_MeasResid = messageIn.TA_AttitudeX_MeasResid;
	messageOut->TA_AttitudeY_MeasResid = messageIn.TA_AttitudeY_MeasResid;
	messageOut->TA_AttitudeZ_MeasResid = messageIn.TA_AttitudeZ_MeasResid;
	messageOut->TA_AttitudeCosZ_MeasResid = messageIn.TA_AttitudeCosZ_MeasResid;
}

void Msg_2429_pub_callback(uint8_t * buffer)
{
	Msg_2429 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2429 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2429);
	ROS_DEBUG("Message 0x2429 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2429_pub_initialized == false){
		init_2429(getRosHandle());}
	// Publish the message
	Msg_2429_pub.publish(msgStruct_2429);
	return;
}
