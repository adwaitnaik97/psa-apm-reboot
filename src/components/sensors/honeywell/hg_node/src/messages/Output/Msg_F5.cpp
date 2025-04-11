#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_F5.h>
#include <hg_node/filter_config_t.h>
hg_node::Msg_F5 msgStruct_F5;

bool Msg_F5_pub_initialized = false;

ros::Publisher Msg_F5_pub;
void init_F5(ros::NodeHandle * n){
	Msg_F5_pub = n->advertise<hg_node::Msg_F5>(MSG_F5_PATH, 5);
	Msg_F5_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_F5_PATH);
	return;
}

void stop_F5(void){
	if (Msg_F5_pub_initialized){
		Msg_F5_pub.shutdown();
		Msg_F5_pub_initialized = false;
		ROS_INFO("0xF5 stopped");
	}
	return;
}

// Msg_F5 to Topic
void convert(Msg_F5 messageIn, hg_node::Msg_F5 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->gyro_range = messageIn.gyro_range;

	messageOut->gyro_config.cutoff_frequency = messageIn.gyro_config.cutoff_frequency;
	messageOut->gyro_config.enabled = messageIn.gyro_config.enabled;
	messageOut->gyro_sample_rate = messageIn.gyro_sample_rate;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_F5
void convert(hg_node::Msg_F5 messageIn, Msg_F5 * messageOut)
{
	messageOut->gyro_range = messageIn.gyro_range;

	messageOut->gyro_config.cutoff_frequency = messageIn.gyro_config.cutoff_frequency;
	messageOut->gyro_config.enabled = messageIn.gyro_config.enabled;
	messageOut->gyro_sample_rate = messageIn.gyro_sample_rate;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_F5_pub_callback(uint8_t * buffer)
{
	Msg_F5 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xF5 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_F5);
	ROS_DEBUG("Message 0xF5 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_F5_pub_initialized == false){
		init_F5(getRosHandle());}
	// Publish the message
	Msg_F5_pub.publish(msgStruct_F5);
	return;
}
