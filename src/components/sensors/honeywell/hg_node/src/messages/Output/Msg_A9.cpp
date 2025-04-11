#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_A9.h>
hg_node::Msg_A9 msgStruct_A9;

bool Msg_A9_pub_initialized = false;

ros::Publisher Msg_A9_pub;
void init_A9(ros::NodeHandle * n){
	Msg_A9_pub = n->advertise<hg_node::Msg_A9>(MSG_A9_PATH, 5);
	Msg_A9_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_A9_PATH);
	return;
}

void stop_A9(void){
	if (Msg_A9_pub_initialized){
		Msg_A9_pub.shutdown();
		Msg_A9_pub_initialized = false;
		ROS_INFO("0xA9 stopped");
	}
	return;
}

// Msg_A9 to Topic
void convert(Msg_A9 messageIn, hg_node::Msg_A9 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->mg1FluxA = messageIn.mg1FluxA;
	messageOut->mg1FluxB = messageIn.mg1FluxB;
	messageOut->mg1FluxC = messageIn.mg1FluxC;
	messageOut->mg1Temperature = messageIn.mg1Temperature;
	messageOut->mg2FluxA = messageIn.mg2FluxA;
	messageOut->mg2FluxB = messageIn.mg2FluxB;
	messageOut->mg2FluxC = messageIn.mg2FluxC;
	messageOut->mg2Temperature = messageIn.mg2Temperature;
	messageOut->mg3FluxA = messageIn.mg3FluxA;
	messageOut->mg3FluxB = messageIn.mg3FluxB;
	messageOut->mg3FluxC = messageIn.mg3FluxC;
	messageOut->mg3Temperature = messageIn.mg3Temperature;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_A9
void convert(hg_node::Msg_A9 messageIn, Msg_A9 * messageOut)
{
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->mg1FluxA = messageIn.mg1FluxA;
	messageOut->mg1FluxB = messageIn.mg1FluxB;
	messageOut->mg1FluxC = messageIn.mg1FluxC;
	messageOut->mg1Temperature = messageIn.mg1Temperature;
	messageOut->mg2FluxA = messageIn.mg2FluxA;
	messageOut->mg2FluxB = messageIn.mg2FluxB;
	messageOut->mg2FluxC = messageIn.mg2FluxC;
	messageOut->mg2Temperature = messageIn.mg2Temperature;
	messageOut->mg3FluxA = messageIn.mg3FluxA;
	messageOut->mg3FluxB = messageIn.mg3FluxB;
	messageOut->mg3FluxC = messageIn.mg3FluxC;
	messageOut->mg3Temperature = messageIn.mg3Temperature;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_A9_pub_callback(uint8_t * buffer)
{
	Msg_A9 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xA9 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_A9);
	ROS_DEBUG("Message 0xA9 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_A9_pub_initialized == false){
		init_A9(getRosHandle());}
	// Publish the message
	Msg_A9_pub.publish(msgStruct_A9);
	return;
}
