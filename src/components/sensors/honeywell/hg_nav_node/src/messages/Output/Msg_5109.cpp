#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5109.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5109 msgStruct_5109;

bool Msg_5109_pub_initialized = false;

ros::Publisher Msg_5109_pub;
void init_5109(ros::NodeHandle * n){
	Msg_5109_pub = n->advertise<hg_nav_node::Msg_5109>(MSG_5109_PATH, 5);
	Msg_5109_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5109_PATH);
	return;
}

void stop_5109(void){
	Msg_5109_pub.shutdown();
	Msg_5109_pub_initialized = false;
	ROS_INFO("0x5109 stopped");
	return;
}

// Msg_5109 to Topic
void convert(Msg_5109 messageIn, hg_nav_node::Msg_5109 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->True_Heading = messageIn.True_Heading;
	messageOut->Roll_Stdv = messageIn.Roll_Stdv;
	messageOut->Pitch_Stdv = messageIn.Pitch_Stdv;
	messageOut->True_Heading_Stdv = messageIn.True_Heading_Stdv;
	messageOut->Data_source = messageIn.Data_source;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
}

// Topic to Msg_5109
void convert(hg_nav_node::Msg_5109 messageIn, Msg_5109 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->True_Heading = messageIn.True_Heading;
	messageOut->Roll_Stdv = messageIn.Roll_Stdv;
	messageOut->Pitch_Stdv = messageIn.Pitch_Stdv;
	messageOut->True_Heading_Stdv = messageIn.True_Heading_Stdv;
	messageOut->Data_source = messageIn.Data_source;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
}

void Msg_5109_pub_callback(uint8_t * buffer)
{
	Msg_5109 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5109 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5109);
	ROS_DEBUG("Message 0x5109 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5109_pub_initialized == false){
		init_5109(getRosHandle());}
	// Publish the message
	Msg_5109_pub.publish(msgStruct_5109);
	return;
}
