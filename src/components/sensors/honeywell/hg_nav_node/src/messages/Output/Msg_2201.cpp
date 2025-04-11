#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2201.h>
#include <hg_nav_node/time_validity_bits_t.h>
hg_nav_node::Msg_2201 msgStruct_2201;

bool Msg_2201_pub_initialized = false;

ros::Publisher Msg_2201_pub;
void init_2201(ros::NodeHandle * n){
	Msg_2201_pub = n->advertise<hg_nav_node::Msg_2201>(MSG_2201_PATH, 5);
	Msg_2201_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2201_PATH);
	return;
}

void stop_2201(void){
	Msg_2201_pub.shutdown();
	Msg_2201_pub_initialized = false;
	ROS_INFO("0x2201 stopped");
	return;
}

// Msg_2201 to Topic
void convert(Msg_2201 messageIn, hg_nav_node::Msg_2201 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->time_validity_bits.gpsTovValid = messageIn.time_validity_bits.gpsTovValid;
	messageOut->time_validity_bits.utcTimeValid = messageIn.time_validity_bits.utcTimeValid;
	messageOut->time_validity_bits.PPS_Mode = messageIn.time_validity_bits.PPS_Mode;
	messageOut->time_validity_bits.gpsTovValid = messageIn.time_validity_bits.gpsTovValid;
	messageOut->time_validity_bits.utcTimeValid = messageIn.time_validity_bits.utcTimeValid;
	messageOut->time_validity_bits.PPS_Mode = messageIn.time_validity_bits.PPS_Mode;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_day_month_year_date = messageIn.utc_day_month_year_date;
	messageOut->utc_day_of_week = messageIn.utc_day_of_week;
	messageOut->utc_day_of_year = messageIn.utc_day_of_year;
	messageOut->utc_day_of_month = messageIn.utc_day_of_month;
	messageOut->utc_month = messageIn.utc_month;
	messageOut->utc_year = messageIn.utc_year;
	messageOut->utc_hour_min_sec = messageIn.utc_hour_min_sec;
	messageOut->utc_second = messageIn.utc_second;
	messageOut->utc_minute = messageIn.utc_minute;
	messageOut->utc_hour = messageIn.utc_hour;
	messageOut->utc_time = messageIn.utc_time;
	messageOut->ins_gps_internal_system_time_pps_in = messageIn.ins_gps_internal_system_time_pps_in;
	messageOut->gps_time_pps_in = messageIn.gps_time_pps_in;
	messageOut->pps_in_count = messageIn.pps_in_count;
}

// Topic to Msg_2201
void convert(hg_nav_node::Msg_2201 messageIn, Msg_2201 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->time_validity_bits.gpsTovValid = messageIn.time_validity_bits.gpsTovValid;
	messageOut->time_validity_bits.utcTimeValid = messageIn.time_validity_bits.utcTimeValid;
	messageOut->time_validity_bits.PPS_Mode = messageIn.time_validity_bits.PPS_Mode;
	messageOut->time_validity_bits.gpsTovValid = messageIn.time_validity_bits.gpsTovValid;
	messageOut->time_validity_bits.utcTimeValid = messageIn.time_validity_bits.utcTimeValid;
	messageOut->time_validity_bits.PPS_Mode = messageIn.time_validity_bits.PPS_Mode;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_day_month_year_date = messageIn.utc_day_month_year_date;
	messageOut->utc_day_of_week = messageIn.utc_day_of_week;
	messageOut->utc_day_of_year = messageIn.utc_day_of_year;
	messageOut->utc_day_of_month = messageIn.utc_day_of_month;
	messageOut->utc_month = messageIn.utc_month;
	messageOut->utc_year = messageIn.utc_year;
	messageOut->utc_hour_min_sec = messageIn.utc_hour_min_sec;
	messageOut->utc_second = messageIn.utc_second;
	messageOut->utc_minute = messageIn.utc_minute;
	messageOut->utc_hour = messageIn.utc_hour;
	messageOut->utc_time = messageIn.utc_time;
	messageOut->ins_gps_internal_system_time_pps_in = messageIn.ins_gps_internal_system_time_pps_in;
	messageOut->gps_time_pps_in = messageIn.gps_time_pps_in;
	messageOut->pps_in_count = messageIn.pps_in_count;
}

void Msg_2201_pub_callback(uint8_t * buffer)
{
	Msg_2201 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2201 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2201);
	ROS_DEBUG("Message 0x2201 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2201_pub_initialized == false){
		init_2201(getRosHandle());}
	// Publish the message
	Msg_2201_pub.publish(msgStruct_2201);
	return;
}
