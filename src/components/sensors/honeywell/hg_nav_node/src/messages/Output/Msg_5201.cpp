#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5201.h>
hg_nav_node::Msg_5201 msgStruct_5201;

bool Msg_5201_pub_initialized = false;

ros::Publisher Msg_5201_pub;
void init_5201(ros::NodeHandle * n){
	Msg_5201_pub = n->advertise<hg_nav_node::Msg_5201>(MSG_5201_PATH, 5);
	Msg_5201_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5201_PATH);
	return;
}

void stop_5201(void){
	Msg_5201_pub.shutdown();
	Msg_5201_pub_initialized = false;
	ROS_INFO("0x5201 stopped");
	return;
}

// Msg_5201 to Topic
void convert(Msg_5201 messageIn, hg_nav_node::Msg_5201 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTovValid = messageIn.gpsTovValid;
	messageOut->gpsToUTCofsetValid = messageIn.gpsToUTCofsetValid;
	messageOut->gps_time_to_utc_offset = messageIn.gps_time_to_utc_offset;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_day_of_week = messageIn.utc_day_of_week;
	messageOut->utc_day_of_year = messageIn.utc_day_of_year;
	messageOut->utc_day_of_month = messageIn.utc_day_of_month;
	messageOut->utc_month = messageIn.utc_month;
	messageOut->utc_year = messageIn.utc_year;
	messageOut->utc_second = messageIn.utc_second;
	messageOut->utc_minute = messageIn.utc_minute;
	messageOut->utc_hour = messageIn.utc_hour;
}

// Topic to Msg_5201
void convert(hg_nav_node::Msg_5201 messageIn, Msg_5201 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTovValid = messageIn.gpsTovValid;
	messageOut->gpsToUTCofsetValid = messageIn.gpsToUTCofsetValid;
	messageOut->gps_time_to_utc_offset = messageIn.gps_time_to_utc_offset;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_day_of_week = messageIn.utc_day_of_week;
	messageOut->utc_day_of_year = messageIn.utc_day_of_year;
	messageOut->utc_day_of_month = messageIn.utc_day_of_month;
	messageOut->utc_month = messageIn.utc_month;
	messageOut->utc_year = messageIn.utc_year;
	messageOut->utc_second = messageIn.utc_second;
	messageOut->utc_minute = messageIn.utc_minute;
	messageOut->utc_hour = messageIn.utc_hour;
}

void Msg_5201_pub_callback(uint8_t * buffer)
{
	Msg_5201 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5201 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5201);
	ROS_DEBUG("Message 0x5201 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5201_pub_initialized == false){
		init_5201(getRosHandle());}
	// Publish the message
	Msg_5201_pub.publish(msgStruct_5201);
	return;
}
