#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6738.h>
hg_nav_node::Msg_6738 msgStruct_6738;

bool Msg_6738_pub_initialized = false;

ros::Publisher Msg_6738_pub;
void init_6738(ros::NodeHandle * n){
	Msg_6738_pub = n->advertise<hg_nav_node::Msg_6738>(MSG_6738_PATH, 5);
	Msg_6738_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6738_PATH);
	return;
}

void stop_6738(void){
	Msg_6738_pub.shutdown();
	Msg_6738_pub_initialized = false;
	ROS_INFO("0x6738 stopped");
	return;
}

// Msg_6738 to Topic
void convert(Msg_6738 messageIn, hg_nav_node::Msg_6738 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->velNormResBeam0BT = messageIn.velNormResBeam0BT;
	messageOut->velNormResBeam0WT = messageIn.velNormResBeam0WT;
	messageOut->velNormResBeam1BT = messageIn.velNormResBeam1BT;
	messageOut->velNormResBeam1WT = messageIn.velNormResBeam1WT;
	messageOut->velNormResBeam2BT = messageIn.velNormResBeam2BT;
	messageOut->velNormResBeam2WT = messageIn.velNormResBeam2WT;
	messageOut->velNormResBeam3BT = messageIn.velNormResBeam3BT;
	messageOut->velNormResBeam3WT = messageIn.velNormResBeam3WT;
	messageOut->sfErrorEstBeam0 = messageIn.sfErrorEstBeam0;
	messageOut->sfErrorEstBeam1 = messageIn.sfErrorEstBeam1;
	messageOut->sfErrorEstBeam2 = messageIn.sfErrorEstBeam2;
	messageOut->sfErrorEstBeam3 = messageIn.sfErrorEstBeam3;
	messageOut->psiYErrorEstBeam0 = messageIn.psiYErrorEstBeam0;
	messageOut->psiYErrorEstBeam1 = messageIn.psiYErrorEstBeam1;
	messageOut->psiYErrorEstBeam2 = messageIn.psiYErrorEstBeam2;
	messageOut->psiYErrorEstBeam3 = messageIn.psiYErrorEstBeam3;
	messageOut->psiZErrorEstBeam0 = messageIn.psiZErrorEstBeam0;
	messageOut->psiZErrorEstBeam1 = messageIn.psiZErrorEstBeam1;
	messageOut->psiZErrorEstBeam2 = messageIn.psiZErrorEstBeam2;
	messageOut->psiZErrorEstBeam3 = messageIn.psiZErrorEstBeam3;
	messageOut->oceanCurErrorEstBeam0 = messageIn.oceanCurErrorEstBeam0;
	messageOut->oceanCurErrorEstBeam1 = messageIn.oceanCurErrorEstBeam1;
	messageOut->oceanCurErrorEstBeam2 = messageIn.oceanCurErrorEstBeam2;
	messageOut->oceanCurErrorEstBeam3 = messageIn.oceanCurErrorEstBeam3;
	messageOut->sfStdvBeam0 = messageIn.sfStdvBeam0;
	messageOut->sfStdvBeam1 = messageIn.sfStdvBeam1;
	messageOut->sfStdvBeam2 = messageIn.sfStdvBeam2;
	messageOut->sfStdvBeam3 = messageIn.sfStdvBeam3;
	messageOut->psiYStdvBeam0 = messageIn.psiYStdvBeam0;
	messageOut->psiYStdvBeam1 = messageIn.psiYStdvBeam1;
	messageOut->psiYStdvBeam2 = messageIn.psiYStdvBeam2;
	messageOut->psiYStdvBeam3 = messageIn.psiYStdvBeam3;
	messageOut->psiZStdvBeam0 = messageIn.psiZStdvBeam0;
	messageOut->psiZStdvBeam1 = messageIn.psiZStdvBeam1;
	messageOut->psiZStdvBeam2 = messageIn.psiZStdvBeam2;
	messageOut->psiZStdvBeam3 = messageIn.psiZStdvBeam3;
	messageOut->oceanCurStdvBeam0 = messageIn.oceanCurStdvBeam0;
	messageOut->oceanCurStdvBeam1 = messageIn.oceanCurStdvBeam1;
	messageOut->oceanCurStdvBeam2 = messageIn.oceanCurStdvBeam2;
	messageOut->oceanCurStdvBeam3 = messageIn.oceanCurStdvBeam3;
}

// Topic to Msg_6738
void convert(hg_nav_node::Msg_6738 messageIn, Msg_6738 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->velNormResBeam0BT = messageIn.velNormResBeam0BT;
	messageOut->velNormResBeam0WT = messageIn.velNormResBeam0WT;
	messageOut->velNormResBeam1BT = messageIn.velNormResBeam1BT;
	messageOut->velNormResBeam1WT = messageIn.velNormResBeam1WT;
	messageOut->velNormResBeam2BT = messageIn.velNormResBeam2BT;
	messageOut->velNormResBeam2WT = messageIn.velNormResBeam2WT;
	messageOut->velNormResBeam3BT = messageIn.velNormResBeam3BT;
	messageOut->velNormResBeam3WT = messageIn.velNormResBeam3WT;
	messageOut->sfErrorEstBeam0 = messageIn.sfErrorEstBeam0;
	messageOut->sfErrorEstBeam1 = messageIn.sfErrorEstBeam1;
	messageOut->sfErrorEstBeam2 = messageIn.sfErrorEstBeam2;
	messageOut->sfErrorEstBeam3 = messageIn.sfErrorEstBeam3;
	messageOut->psiYErrorEstBeam0 = messageIn.psiYErrorEstBeam0;
	messageOut->psiYErrorEstBeam1 = messageIn.psiYErrorEstBeam1;
	messageOut->psiYErrorEstBeam2 = messageIn.psiYErrorEstBeam2;
	messageOut->psiYErrorEstBeam3 = messageIn.psiYErrorEstBeam3;
	messageOut->psiZErrorEstBeam0 = messageIn.psiZErrorEstBeam0;
	messageOut->psiZErrorEstBeam1 = messageIn.psiZErrorEstBeam1;
	messageOut->psiZErrorEstBeam2 = messageIn.psiZErrorEstBeam2;
	messageOut->psiZErrorEstBeam3 = messageIn.psiZErrorEstBeam3;
	messageOut->oceanCurErrorEstBeam0 = messageIn.oceanCurErrorEstBeam0;
	messageOut->oceanCurErrorEstBeam1 = messageIn.oceanCurErrorEstBeam1;
	messageOut->oceanCurErrorEstBeam2 = messageIn.oceanCurErrorEstBeam2;
	messageOut->oceanCurErrorEstBeam3 = messageIn.oceanCurErrorEstBeam3;
	messageOut->sfStdvBeam0 = messageIn.sfStdvBeam0;
	messageOut->sfStdvBeam1 = messageIn.sfStdvBeam1;
	messageOut->sfStdvBeam2 = messageIn.sfStdvBeam2;
	messageOut->sfStdvBeam3 = messageIn.sfStdvBeam3;
	messageOut->psiYStdvBeam0 = messageIn.psiYStdvBeam0;
	messageOut->psiYStdvBeam1 = messageIn.psiYStdvBeam1;
	messageOut->psiYStdvBeam2 = messageIn.psiYStdvBeam2;
	messageOut->psiYStdvBeam3 = messageIn.psiYStdvBeam3;
	messageOut->psiZStdvBeam0 = messageIn.psiZStdvBeam0;
	messageOut->psiZStdvBeam1 = messageIn.psiZStdvBeam1;
	messageOut->psiZStdvBeam2 = messageIn.psiZStdvBeam2;
	messageOut->psiZStdvBeam3 = messageIn.psiZStdvBeam3;
	messageOut->oceanCurStdvBeam0 = messageIn.oceanCurStdvBeam0;
	messageOut->oceanCurStdvBeam1 = messageIn.oceanCurStdvBeam1;
	messageOut->oceanCurStdvBeam2 = messageIn.oceanCurStdvBeam2;
	messageOut->oceanCurStdvBeam3 = messageIn.oceanCurStdvBeam3;
}

void Msg_6738_pub_callback(uint8_t * buffer)
{
	Msg_6738 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6738 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6738);
	ROS_DEBUG("Message 0x6738 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6738_pub_initialized == false){
		init_6738(getRosHandle());}
	// Publish the message
	Msg_6738_pub.publish(msgStruct_6738);
	return;
}
