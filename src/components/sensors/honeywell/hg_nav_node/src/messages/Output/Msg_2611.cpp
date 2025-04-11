#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2611.h>
hg_nav_node::Msg_2611 msgStruct_2611;

bool Msg_2611_pub_initialized = false;

ros::Publisher Msg_2611_pub;
void init_2611(ros::NodeHandle * n){
	Msg_2611_pub = n->advertise<hg_nav_node::Msg_2611>(MSG_2611_PATH, 5);
	Msg_2611_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2611_PATH);
	return;
}

void stop_2611(void){
	Msg_2611_pub.shutdown();
	Msg_2611_pub_initialized = false;
	ROS_INFO("0x2611 stopped");
	return;
}

// Msg_2611 to Topic
void convert(Msg_2611 messageIn, hg_nav_node::Msg_2611 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Last_Message = messageIn.Last_Message;
	messageOut->messageNumber_a = messageIn.messageNumber_a;
	messageOut->faultID_a = messageIn.faultID_a;
	messageOut->BITmode_a = messageIn.BITmode_a;
	messageOut->systemMode_a = messageIn.systemMode_a;
	messageOut->powerCycleCount_a = messageIn.powerCycleCount_a;
	messageOut->faultValue_a = messageIn.faultValue_a;
	messageOut->channelSubsystem_a = messageIn.channelSubsystem_a;
	messageOut->eti_a = messageIn.eti_a;
	messageOut->angularRateX_a = messageIn.angularRateX_a;
	messageOut->angularRateY_a = messageIn.angularRateY_a;
	messageOut->angularRateZ_a = messageIn.angularRateZ_a;
	messageOut->accelerationX_a = messageIn.accelerationX_a;
	messageOut->accelerationY_a = messageIn.accelerationY_a;
	messageOut->accelerationZ_a = messageIn.accelerationZ_a;
	messageOut->velocityX_a = messageIn.velocityX_a;
	messageOut->velocityY_a = messageIn.velocityY_a;
	messageOut->velocityZ_a = messageIn.velocityZ_a;
	messageOut->insTemperature_a = messageIn.insTemperature_a;
	messageOut->altitude_a = messageIn.altitude_a;
	messageOut->messageNumber_b = messageIn.messageNumber_b;
	messageOut->faultID_b = messageIn.faultID_b;
	messageOut->BITmode_b = messageIn.BITmode_b;
	messageOut->systemMode_b = messageIn.systemMode_b;
	messageOut->powerCycleCount_b = messageIn.powerCycleCount_b;
	messageOut->faultValue_b = messageIn.faultValue_b;
	messageOut->channelSubsystem_b = messageIn.channelSubsystem_b;
	messageOut->eti_b = messageIn.eti_b;
	messageOut->angularRateX_b = messageIn.angularRateX_b;
	messageOut->angularRateY_b = messageIn.angularRateY_b;
	messageOut->angularRateZ_b = messageIn.angularRateZ_b;
	messageOut->accelerationX_b = messageIn.accelerationX_b;
	messageOut->accelerationY_b = messageIn.accelerationY_b;
	messageOut->accelerationZ_b = messageIn.accelerationZ_b;
	messageOut->velocityX_b = messageIn.velocityX_b;
	messageOut->velocityY_b = messageIn.velocityY_b;
	messageOut->velocityZ_b = messageIn.velocityZ_b;
	messageOut->insTemperature_b = messageIn.insTemperature_b;
	messageOut->altitude_b = messageIn.altitude_b;
}

// Topic to Msg_2611
void convert(hg_nav_node::Msg_2611 messageIn, Msg_2611 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Last_Message = messageIn.Last_Message;
	messageOut->messageNumber_a = messageIn.messageNumber_a;
	messageOut->faultID_a = messageIn.faultID_a;
	messageOut->BITmode_a = messageIn.BITmode_a;
	messageOut->systemMode_a = messageIn.systemMode_a;
	messageOut->powerCycleCount_a = messageIn.powerCycleCount_a;
	messageOut->faultValue_a = messageIn.faultValue_a;
	messageOut->channelSubsystem_a = messageIn.channelSubsystem_a;
	messageOut->eti_a = messageIn.eti_a;
	messageOut->angularRateX_a = messageIn.angularRateX_a;
	messageOut->angularRateY_a = messageIn.angularRateY_a;
	messageOut->angularRateZ_a = messageIn.angularRateZ_a;
	messageOut->accelerationX_a = messageIn.accelerationX_a;
	messageOut->accelerationY_a = messageIn.accelerationY_a;
	messageOut->accelerationZ_a = messageIn.accelerationZ_a;
	messageOut->velocityX_a = messageIn.velocityX_a;
	messageOut->velocityY_a = messageIn.velocityY_a;
	messageOut->velocityZ_a = messageIn.velocityZ_a;
	messageOut->insTemperature_a = messageIn.insTemperature_a;
	messageOut->altitude_a = messageIn.altitude_a;
	messageOut->messageNumber_b = messageIn.messageNumber_b;
	messageOut->faultID_b = messageIn.faultID_b;
	messageOut->BITmode_b = messageIn.BITmode_b;
	messageOut->systemMode_b = messageIn.systemMode_b;
	messageOut->powerCycleCount_b = messageIn.powerCycleCount_b;
	messageOut->faultValue_b = messageIn.faultValue_b;
	messageOut->channelSubsystem_b = messageIn.channelSubsystem_b;
	messageOut->eti_b = messageIn.eti_b;
	messageOut->angularRateX_b = messageIn.angularRateX_b;
	messageOut->angularRateY_b = messageIn.angularRateY_b;
	messageOut->angularRateZ_b = messageIn.angularRateZ_b;
	messageOut->accelerationX_b = messageIn.accelerationX_b;
	messageOut->accelerationY_b = messageIn.accelerationY_b;
	messageOut->accelerationZ_b = messageIn.accelerationZ_b;
	messageOut->velocityX_b = messageIn.velocityX_b;
	messageOut->velocityY_b = messageIn.velocityY_b;
	messageOut->velocityZ_b = messageIn.velocityZ_b;
	messageOut->insTemperature_b = messageIn.insTemperature_b;
	messageOut->altitude_b = messageIn.altitude_b;
}

void Msg_2611_pub_callback(uint8_t * buffer)
{
	Msg_2611 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2611 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2611);
	ROS_DEBUG("Message 0x2611 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2611_pub_initialized == false){
		init_2611(getRosHandle());}
	// Publish the message
	Msg_2611_pub.publish(msgStruct_2611);
	return;
}
