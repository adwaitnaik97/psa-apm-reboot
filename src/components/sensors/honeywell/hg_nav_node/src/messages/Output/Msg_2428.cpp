#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2428.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2428 msgStruct_2428;

bool Msg_2428_pub_initialized = false;

ros::Publisher Msg_2428_pub;
void init_2428(ros::NodeHandle * n){
	Msg_2428_pub = n->advertise<hg_nav_node::Msg_2428>(MSG_2428_PATH, 5);
	Msg_2428_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2428_PATH);
	return;
}

void stop_2428(void){
	Msg_2428_pub.shutdown();
	Msg_2428_pub_initialized = false;
	ROS_INFO("0x2428 stopped");
	return;
}

// Msg_2428 to Topic
void convert(Msg_2428 messageIn, hg_nav_node::Msg_2428 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->GNSS_Position_X = messageIn.GNSS_Position_X;
	messageOut->GNSS_Position_Y = messageIn.GNSS_Position_Y;
	messageOut->GNSS_Position_Z = messageIn.GNSS_Position_Z;
	messageOut->GNSS_Velocity_X = messageIn.GNSS_Velocity_X;
	messageOut->GNSS_Velocity_Y = messageIn.GNSS_Velocity_Y;
	messageOut->GNSS_Velocity_Z = messageIn.GNSS_Velocity_Z;
	messageOut->Number_of_PR_channels = messageIn.Number_of_PR_channels;
	messageOut->GPS_PR_Channel_1 = messageIn.GPS_PR_Channel_1;
	messageOut->GPS_PR_Channel_2 = messageIn.GPS_PR_Channel_2;
	messageOut->GPS_PR_Channel_3 = messageIn.GPS_PR_Channel_3;
	messageOut->GPS_PR_Channel_4 = messageIn.GPS_PR_Channel_4;
	messageOut->GPS_PR_Channel_5 = messageIn.GPS_PR_Channel_5;
	messageOut->GPS_PR_Channel_6 = messageIn.GPS_PR_Channel_6;
	messageOut->GPS_PR_Channel_7 = messageIn.GPS_PR_Channel_7;
	messageOut->GPS_PR_Channel_8 = messageIn.GPS_PR_Channel_8;
	messageOut->GPS_PR_Channel_9 = messageIn.GPS_PR_Channel_9;
	messageOut->GPS_PR_Channel_10 = messageIn.GPS_PR_Channel_10;
	messageOut->GPS_PR_Channel_11 = messageIn.GPS_PR_Channel_11;
	messageOut->GPS_PR_Channel_12 = messageIn.GPS_PR_Channel_12;
	messageOut->Number_of_DR_channels = messageIn.Number_of_DR_channels;
	messageOut->GPS_DR_Channel_1 = messageIn.GPS_DR_Channel_1;
	messageOut->GPS_DR_Channel_2 = messageIn.GPS_DR_Channel_2;
	messageOut->GPS_DR_Channel_3 = messageIn.GPS_DR_Channel_3;
	messageOut->GPS_DR_Channel_4 = messageIn.GPS_DR_Channel_4;
	messageOut->GPS_DR_Channel_5 = messageIn.GPS_DR_Channel_5;
	messageOut->GPS_DR_Channel_6 = messageIn.GPS_DR_Channel_6;
	messageOut->GPS_DR_Channel_7 = messageIn.GPS_DR_Channel_7;
	messageOut->GPS_DR_Channel_8 = messageIn.GPS_DR_Channel_8;
	messageOut->GPS_DR_Channel_9 = messageIn.GPS_DR_Channel_9;
	messageOut->GPS_DR_Channel_10 = messageIn.GPS_DR_Channel_10;
	messageOut->GPS_DR_Channel_11 = messageIn.GPS_DR_Channel_11;
	messageOut->GPS_DR_Channel_12 = messageIn.GPS_DR_Channel_12;
}

// Topic to Msg_2428
void convert(hg_nav_node::Msg_2428 messageIn, Msg_2428 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->GNSS_Position_X = messageIn.GNSS_Position_X;
	messageOut->GNSS_Position_Y = messageIn.GNSS_Position_Y;
	messageOut->GNSS_Position_Z = messageIn.GNSS_Position_Z;
	messageOut->GNSS_Velocity_X = messageIn.GNSS_Velocity_X;
	messageOut->GNSS_Velocity_Y = messageIn.GNSS_Velocity_Y;
	messageOut->GNSS_Velocity_Z = messageIn.GNSS_Velocity_Z;
	messageOut->Number_of_PR_channels = messageIn.Number_of_PR_channels;
	messageOut->GPS_PR_Channel_1 = messageIn.GPS_PR_Channel_1;
	messageOut->GPS_PR_Channel_2 = messageIn.GPS_PR_Channel_2;
	messageOut->GPS_PR_Channel_3 = messageIn.GPS_PR_Channel_3;
	messageOut->GPS_PR_Channel_4 = messageIn.GPS_PR_Channel_4;
	messageOut->GPS_PR_Channel_5 = messageIn.GPS_PR_Channel_5;
	messageOut->GPS_PR_Channel_6 = messageIn.GPS_PR_Channel_6;
	messageOut->GPS_PR_Channel_7 = messageIn.GPS_PR_Channel_7;
	messageOut->GPS_PR_Channel_8 = messageIn.GPS_PR_Channel_8;
	messageOut->GPS_PR_Channel_9 = messageIn.GPS_PR_Channel_9;
	messageOut->GPS_PR_Channel_10 = messageIn.GPS_PR_Channel_10;
	messageOut->GPS_PR_Channel_11 = messageIn.GPS_PR_Channel_11;
	messageOut->GPS_PR_Channel_12 = messageIn.GPS_PR_Channel_12;
	messageOut->Number_of_DR_channels = messageIn.Number_of_DR_channels;
	messageOut->GPS_DR_Channel_1 = messageIn.GPS_DR_Channel_1;
	messageOut->GPS_DR_Channel_2 = messageIn.GPS_DR_Channel_2;
	messageOut->GPS_DR_Channel_3 = messageIn.GPS_DR_Channel_3;
	messageOut->GPS_DR_Channel_4 = messageIn.GPS_DR_Channel_4;
	messageOut->GPS_DR_Channel_5 = messageIn.GPS_DR_Channel_5;
	messageOut->GPS_DR_Channel_6 = messageIn.GPS_DR_Channel_6;
	messageOut->GPS_DR_Channel_7 = messageIn.GPS_DR_Channel_7;
	messageOut->GPS_DR_Channel_8 = messageIn.GPS_DR_Channel_8;
	messageOut->GPS_DR_Channel_9 = messageIn.GPS_DR_Channel_9;
	messageOut->GPS_DR_Channel_10 = messageIn.GPS_DR_Channel_10;
	messageOut->GPS_DR_Channel_11 = messageIn.GPS_DR_Channel_11;
	messageOut->GPS_DR_Channel_12 = messageIn.GPS_DR_Channel_12;
}

void Msg_2428_pub_callback(uint8_t * buffer)
{
	Msg_2428 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2428 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2428);
	ROS_DEBUG("Message 0x2428 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2428_pub_initialized == false){
		init_2428(getRosHandle());}
	// Publish the message
	Msg_2428_pub.publish(msgStruct_2428);
	return;
}
