#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F9.h>
hg_node::Msg_F9 msgStruct_F9;

bool Msg_F9_pub_initialized = false;

ros::Publisher Msg_F9_pub;
void init_F9(ros::NodeHandle * n){
	Msg_F9_pub = n->advertise<hg_node::Msg_F9>(MSG_F9_PATH, 5);
	Msg_F9_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F9_PATH);
	return;
}

void stop_F9(void){
	if (Msg_F9_pub_initialized){
		Msg_F9_pub.shutdown();
		Msg_F9_pub_initialized = false;
		ROS_INFO("0xF9 stopped");
	}
	return;
}

// Msg_F9 to Topic
void convert(Msg_F9 messageIn, hg_node::Msg_F9 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->percent_fr = messageIn.percent_fr;
	messageOut->percent_ffr = messageIn.percent_ffr;
	messageOut->percent_cr = messageIn.percent_cr;
	messageOut->percent_gr = messageIn.percent_gr;
	messageOut->percent_100_fr = messageIn.percent_100_fr;
	messageOut->percent_10_fr = messageIn.percent_10_fr;
	messageOut->percent_1_fr = messageIn.percent_1_fr;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F9
void convert(hg_node::Msg_F9 messageIn, Msg_F9 * messageOut)
{
	messageOut->percent_fr = messageIn.percent_fr;
	messageOut->percent_ffr = messageIn.percent_ffr;
	messageOut->percent_cr = messageIn.percent_cr;
	messageOut->percent_gr = messageIn.percent_gr;
	messageOut->percent_100_fr = messageIn.percent_100_fr;
	messageOut->percent_10_fr = messageIn.percent_10_fr;
	messageOut->percent_1_fr = messageIn.percent_1_fr;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F9_pub_callback(uint8_t * buffer)
{
	Msg_F9 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF9 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F9);
	ROS_DEBUG("Message 0xF9 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F9_pub_initialized == false){
		init_F9(getRosHandle());}
	// Publish the message
	Msg_F9_pub.publish(msgStruct_F9);
	return;
}
