#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2427.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2427 msgStruct_2427;

bool Msg_2427_pub_initialized = false;

ros::Publisher Msg_2427_pub;
void init_2427(ros::NodeHandle * n){
	Msg_2427_pub = n->advertise<hg_nav_node::Msg_2427>(MSG_2427_PATH, 5);
	Msg_2427_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2427_PATH);
	return;
}

void stop_2427(void){
	Msg_2427_pub.shutdown();
	Msg_2427_pub_initialized = false;
	ROS_INFO("0x2427 stopped");
	return;
}

// Msg_2427 to Topic
void convert(Msg_2427 messageIn, hg_nav_node::Msg_2427 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->MagBiasErrorEstimate = messageIn.MagBiasErrorEstimate;
	messageOut->MagBiasErrorStdv = messageIn.MagBiasErrorStdv;
	messageOut->AttZNormalizedMeasResid = messageIn.AttZNormalizedMeasResid;
	messageOut->AttCosZNormalizedMeasResid = messageIn.AttCosZNormalizedMeasResid;
}

// Topic to Msg_2427
void convert(hg_nav_node::Msg_2427 messageIn, Msg_2427 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->MagBiasErrorEstimate = messageIn.MagBiasErrorEstimate;
	messageOut->MagBiasErrorStdv = messageIn.MagBiasErrorStdv;
	messageOut->AttZNormalizedMeasResid = messageIn.AttZNormalizedMeasResid;
	messageOut->AttCosZNormalizedMeasResid = messageIn.AttCosZNormalizedMeasResid;
}

void Msg_2427_pub_callback(uint8_t * buffer)
{
	Msg_2427 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2427 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2427);
	ROS_DEBUG("Message 0x2427 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2427_pub_initialized == false){
		init_2427(getRosHandle());}
	// Publish the message
	Msg_2427_pub.publish(msgStruct_2427);
	return;
}
