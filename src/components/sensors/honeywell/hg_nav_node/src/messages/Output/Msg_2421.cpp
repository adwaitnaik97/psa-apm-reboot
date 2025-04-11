#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2421.h>
#include <hg_nav_node/gnss_pvt_aiding_status_t.h>
#include <hg_nav_node/gps_aiding_status_t.h>
#include <hg_nav_node/misc_aiding_status_t.h>
#include <hg_nav_node/ta_aiding_status_t.h>
hg_nav_node::Msg_2421 msgStruct_2421;

bool Msg_2421_pub_initialized = false;

ros::Publisher Msg_2421_pub;
void init_2421(ros::NodeHandle * n){
	Msg_2421_pub = n->advertise<hg_nav_node::Msg_2421>(MSG_2421_PATH, 5);
	Msg_2421_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2421_PATH);
	return;
}

void stop_2421(void){
	Msg_2421_pub.shutdown();
	Msg_2421_pub_initialized = false;
	ROS_INFO("0x2421 stopped");
	return;
}

// Msg_2421 to Topic
void convert(Msg_2421 messageIn, hg_nav_node::Msg_2421 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

	messageOut->gps_pr_aiding_status.channel_1 = messageIn.gps_pr_aiding_status.channel_1;
	messageOut->gps_pr_aiding_status.channel_2 = messageIn.gps_pr_aiding_status.channel_2;
	messageOut->gps_pr_aiding_status.channel_3 = messageIn.gps_pr_aiding_status.channel_3;
	messageOut->gps_pr_aiding_status.channel_4 = messageIn.gps_pr_aiding_status.channel_4;
	messageOut->gps_pr_aiding_status.channel_5 = messageIn.gps_pr_aiding_status.channel_5;
	messageOut->gps_pr_aiding_status.channel_6 = messageIn.gps_pr_aiding_status.channel_6;
	messageOut->gps_pr_aiding_status.channel_7 = messageIn.gps_pr_aiding_status.channel_7;
	messageOut->gps_pr_aiding_status.channel_8 = messageIn.gps_pr_aiding_status.channel_8;
	messageOut->gps_pr_aiding_status.channel_9 = messageIn.gps_pr_aiding_status.channel_9;
	messageOut->gps_pr_aiding_status.channel_10 = messageIn.gps_pr_aiding_status.channel_10;
	messageOut->gps_pr_aiding_status.channel_11 = messageIn.gps_pr_aiding_status.channel_11;
	messageOut->gps_pr_aiding_status.channel_12 = messageIn.gps_pr_aiding_status.channel_12;
	messageOut->gps_pr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_pr_aiding_status.TimeBiasRepartitionEvent;
	messageOut->gps_pr_aiding_status.channel_1 = messageIn.gps_pr_aiding_status.channel_1;
	messageOut->gps_pr_aiding_status.channel_2 = messageIn.gps_pr_aiding_status.channel_2;
	messageOut->gps_pr_aiding_status.channel_3 = messageIn.gps_pr_aiding_status.channel_3;
	messageOut->gps_pr_aiding_status.channel_4 = messageIn.gps_pr_aiding_status.channel_4;
	messageOut->gps_pr_aiding_status.channel_5 = messageIn.gps_pr_aiding_status.channel_5;
	messageOut->gps_pr_aiding_status.channel_6 = messageIn.gps_pr_aiding_status.channel_6;
	messageOut->gps_pr_aiding_status.channel_7 = messageIn.gps_pr_aiding_status.channel_7;
	messageOut->gps_pr_aiding_status.channel_8 = messageIn.gps_pr_aiding_status.channel_8;
	messageOut->gps_pr_aiding_status.channel_9 = messageIn.gps_pr_aiding_status.channel_9;
	messageOut->gps_pr_aiding_status.channel_10 = messageIn.gps_pr_aiding_status.channel_10;
	messageOut->gps_pr_aiding_status.channel_11 = messageIn.gps_pr_aiding_status.channel_11;
	messageOut->gps_pr_aiding_status.channel_12 = messageIn.gps_pr_aiding_status.channel_12;
	messageOut->gps_pr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_pr_aiding_status.TimeBiasRepartitionEvent;

	messageOut->gps_dr_aiding_status.channel_1 = messageIn.gps_dr_aiding_status.channel_1;
	messageOut->gps_dr_aiding_status.channel_2 = messageIn.gps_dr_aiding_status.channel_2;
	messageOut->gps_dr_aiding_status.channel_3 = messageIn.gps_dr_aiding_status.channel_3;
	messageOut->gps_dr_aiding_status.channel_4 = messageIn.gps_dr_aiding_status.channel_4;
	messageOut->gps_dr_aiding_status.channel_5 = messageIn.gps_dr_aiding_status.channel_5;
	messageOut->gps_dr_aiding_status.channel_6 = messageIn.gps_dr_aiding_status.channel_6;
	messageOut->gps_dr_aiding_status.channel_7 = messageIn.gps_dr_aiding_status.channel_7;
	messageOut->gps_dr_aiding_status.channel_8 = messageIn.gps_dr_aiding_status.channel_8;
	messageOut->gps_dr_aiding_status.channel_9 = messageIn.gps_dr_aiding_status.channel_9;
	messageOut->gps_dr_aiding_status.channel_10 = messageIn.gps_dr_aiding_status.channel_10;
	messageOut->gps_dr_aiding_status.channel_11 = messageIn.gps_dr_aiding_status.channel_11;
	messageOut->gps_dr_aiding_status.channel_12 = messageIn.gps_dr_aiding_status.channel_12;
	messageOut->gps_dr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_dr_aiding_status.TimeBiasRepartitionEvent;
	messageOut->gps_dr_aiding_status.channel_1 = messageIn.gps_dr_aiding_status.channel_1;
	messageOut->gps_dr_aiding_status.channel_2 = messageIn.gps_dr_aiding_status.channel_2;
	messageOut->gps_dr_aiding_status.channel_3 = messageIn.gps_dr_aiding_status.channel_3;
	messageOut->gps_dr_aiding_status.channel_4 = messageIn.gps_dr_aiding_status.channel_4;
	messageOut->gps_dr_aiding_status.channel_5 = messageIn.gps_dr_aiding_status.channel_5;
	messageOut->gps_dr_aiding_status.channel_6 = messageIn.gps_dr_aiding_status.channel_6;
	messageOut->gps_dr_aiding_status.channel_7 = messageIn.gps_dr_aiding_status.channel_7;
	messageOut->gps_dr_aiding_status.channel_8 = messageIn.gps_dr_aiding_status.channel_8;
	messageOut->gps_dr_aiding_status.channel_9 = messageIn.gps_dr_aiding_status.channel_9;
	messageOut->gps_dr_aiding_status.channel_10 = messageIn.gps_dr_aiding_status.channel_10;
	messageOut->gps_dr_aiding_status.channel_11 = messageIn.gps_dr_aiding_status.channel_11;
	messageOut->gps_dr_aiding_status.channel_12 = messageIn.gps_dr_aiding_status.channel_12;
	messageOut->gps_dr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_dr_aiding_status.TimeBiasRepartitionEvent;

	messageOut->ta_aiding_status.position_ecef_X = messageIn.ta_aiding_status.position_ecef_X;
	messageOut->ta_aiding_status.position_ecef_Y = messageIn.ta_aiding_status.position_ecef_Y;
	messageOut->ta_aiding_status.position_ecef_Z = messageIn.ta_aiding_status.position_ecef_Z;
	messageOut->ta_aiding_status.velocity_ecef_X = messageIn.ta_aiding_status.velocity_ecef_X;
	messageOut->ta_aiding_status.velocity_ecef_Y = messageIn.ta_aiding_status.velocity_ecef_Y;
	messageOut->ta_aiding_status.velocity_ecef_Z = messageIn.ta_aiding_status.velocity_ecef_Z;
	messageOut->ta_aiding_status.attitude_roll = messageIn.ta_aiding_status.attitude_roll;
	messageOut->ta_aiding_status.attitude_pitch = messageIn.ta_aiding_status.attitude_pitch;
	messageOut->ta_aiding_status.attitude_heading = messageIn.ta_aiding_status.attitude_heading;
	messageOut->ta_aiding_status.position_ecef_X = messageIn.ta_aiding_status.position_ecef_X;
	messageOut->ta_aiding_status.position_ecef_Y = messageIn.ta_aiding_status.position_ecef_Y;
	messageOut->ta_aiding_status.position_ecef_Z = messageIn.ta_aiding_status.position_ecef_Z;
	messageOut->ta_aiding_status.velocity_ecef_X = messageIn.ta_aiding_status.velocity_ecef_X;
	messageOut->ta_aiding_status.velocity_ecef_Y = messageIn.ta_aiding_status.velocity_ecef_Y;
	messageOut->ta_aiding_status.velocity_ecef_Z = messageIn.ta_aiding_status.velocity_ecef_Z;
	messageOut->ta_aiding_status.attitude_roll = messageIn.ta_aiding_status.attitude_roll;
	messageOut->ta_aiding_status.attitude_pitch = messageIn.ta_aiding_status.attitude_pitch;
	messageOut->ta_aiding_status.attitude_heading = messageIn.ta_aiding_status.attitude_heading;

	messageOut->gnss_pvt_aiding_status.latitude = messageIn.gnss_pvt_aiding_status.latitude;
	messageOut->gnss_pvt_aiding_status.longitude = messageIn.gnss_pvt_aiding_status.longitude;
	messageOut->gnss_pvt_aiding_status.altitude = messageIn.gnss_pvt_aiding_status.altitude;
	messageOut->gnss_pvt_aiding_status.velocity_north = messageIn.gnss_pvt_aiding_status.velocity_north;
	messageOut->gnss_pvt_aiding_status.velocity_east = messageIn.gnss_pvt_aiding_status.velocity_east;
	messageOut->gnss_pvt_aiding_status.velocity_down = messageIn.gnss_pvt_aiding_status.velocity_down;
	messageOut->gnss_pvt_aiding_status.latitude = messageIn.gnss_pvt_aiding_status.latitude;
	messageOut->gnss_pvt_aiding_status.longitude = messageIn.gnss_pvt_aiding_status.longitude;
	messageOut->gnss_pvt_aiding_status.altitude = messageIn.gnss_pvt_aiding_status.altitude;
	messageOut->gnss_pvt_aiding_status.velocity_north = messageIn.gnss_pvt_aiding_status.velocity_north;
	messageOut->gnss_pvt_aiding_status.velocity_east = messageIn.gnss_pvt_aiding_status.velocity_east;
	messageOut->gnss_pvt_aiding_status.velocity_down = messageIn.gnss_pvt_aiding_status.velocity_down;

	messageOut->misc_aiding_status.zero_velocity_x = messageIn.misc_aiding_status.zero_velocity_x;
	messageOut->misc_aiding_status.zero_velocity_y = messageIn.misc_aiding_status.zero_velocity_y;
	messageOut->misc_aiding_status.zero_velocity_z = messageIn.misc_aiding_status.zero_velocity_z;
	messageOut->misc_aiding_status.magnetic_heading = messageIn.misc_aiding_status.magnetic_heading;
	messageOut->misc_aiding_status.zero_heading_change = messageIn.misc_aiding_status.zero_heading_change;
	messageOut->misc_aiding_status.barometric_altitude = messageIn.misc_aiding_status.barometric_altitude;
	messageOut->misc_aiding_status.zero_velocity_x = messageIn.misc_aiding_status.zero_velocity_x;
	messageOut->misc_aiding_status.zero_velocity_y = messageIn.misc_aiding_status.zero_velocity_y;
	messageOut->misc_aiding_status.zero_velocity_z = messageIn.misc_aiding_status.zero_velocity_z;
	messageOut->misc_aiding_status.magnetic_heading = messageIn.misc_aiding_status.magnetic_heading;
	messageOut->misc_aiding_status.zero_heading_change = messageIn.misc_aiding_status.zero_heading_change;
	messageOut->misc_aiding_status.barometric_altitude = messageIn.misc_aiding_status.barometric_altitude;
	messageOut->Available_Measurements = messageIn.Available_Measurements;
	messageOut->Successfully_Processed = messageIn.Successfully_Processed;
	messageOut->Failed_Processing = messageIn.Failed_Processing;
}

// Topic to Msg_2421
void convert(hg_nav_node::Msg_2421 messageIn, Msg_2421 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

	messageOut->gps_pr_aiding_status.channel_1 = messageIn.gps_pr_aiding_status.channel_1;
	messageOut->gps_pr_aiding_status.channel_2 = messageIn.gps_pr_aiding_status.channel_2;
	messageOut->gps_pr_aiding_status.channel_3 = messageIn.gps_pr_aiding_status.channel_3;
	messageOut->gps_pr_aiding_status.channel_4 = messageIn.gps_pr_aiding_status.channel_4;
	messageOut->gps_pr_aiding_status.channel_5 = messageIn.gps_pr_aiding_status.channel_5;
	messageOut->gps_pr_aiding_status.channel_6 = messageIn.gps_pr_aiding_status.channel_6;
	messageOut->gps_pr_aiding_status.channel_7 = messageIn.gps_pr_aiding_status.channel_7;
	messageOut->gps_pr_aiding_status.channel_8 = messageIn.gps_pr_aiding_status.channel_8;
	messageOut->gps_pr_aiding_status.channel_9 = messageIn.gps_pr_aiding_status.channel_9;
	messageOut->gps_pr_aiding_status.channel_10 = messageIn.gps_pr_aiding_status.channel_10;
	messageOut->gps_pr_aiding_status.channel_11 = messageIn.gps_pr_aiding_status.channel_11;
	messageOut->gps_pr_aiding_status.channel_12 = messageIn.gps_pr_aiding_status.channel_12;
	messageOut->gps_pr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_pr_aiding_status.TimeBiasRepartitionEvent;
	messageOut->gps_pr_aiding_status.channel_1 = messageIn.gps_pr_aiding_status.channel_1;
	messageOut->gps_pr_aiding_status.channel_2 = messageIn.gps_pr_aiding_status.channel_2;
	messageOut->gps_pr_aiding_status.channel_3 = messageIn.gps_pr_aiding_status.channel_3;
	messageOut->gps_pr_aiding_status.channel_4 = messageIn.gps_pr_aiding_status.channel_4;
	messageOut->gps_pr_aiding_status.channel_5 = messageIn.gps_pr_aiding_status.channel_5;
	messageOut->gps_pr_aiding_status.channel_6 = messageIn.gps_pr_aiding_status.channel_6;
	messageOut->gps_pr_aiding_status.channel_7 = messageIn.gps_pr_aiding_status.channel_7;
	messageOut->gps_pr_aiding_status.channel_8 = messageIn.gps_pr_aiding_status.channel_8;
	messageOut->gps_pr_aiding_status.channel_9 = messageIn.gps_pr_aiding_status.channel_9;
	messageOut->gps_pr_aiding_status.channel_10 = messageIn.gps_pr_aiding_status.channel_10;
	messageOut->gps_pr_aiding_status.channel_11 = messageIn.gps_pr_aiding_status.channel_11;
	messageOut->gps_pr_aiding_status.channel_12 = messageIn.gps_pr_aiding_status.channel_12;
	messageOut->gps_pr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_pr_aiding_status.TimeBiasRepartitionEvent;

	messageOut->gps_dr_aiding_status.channel_1 = messageIn.gps_dr_aiding_status.channel_1;
	messageOut->gps_dr_aiding_status.channel_2 = messageIn.gps_dr_aiding_status.channel_2;
	messageOut->gps_dr_aiding_status.channel_3 = messageIn.gps_dr_aiding_status.channel_3;
	messageOut->gps_dr_aiding_status.channel_4 = messageIn.gps_dr_aiding_status.channel_4;
	messageOut->gps_dr_aiding_status.channel_5 = messageIn.gps_dr_aiding_status.channel_5;
	messageOut->gps_dr_aiding_status.channel_6 = messageIn.gps_dr_aiding_status.channel_6;
	messageOut->gps_dr_aiding_status.channel_7 = messageIn.gps_dr_aiding_status.channel_7;
	messageOut->gps_dr_aiding_status.channel_8 = messageIn.gps_dr_aiding_status.channel_8;
	messageOut->gps_dr_aiding_status.channel_9 = messageIn.gps_dr_aiding_status.channel_9;
	messageOut->gps_dr_aiding_status.channel_10 = messageIn.gps_dr_aiding_status.channel_10;
	messageOut->gps_dr_aiding_status.channel_11 = messageIn.gps_dr_aiding_status.channel_11;
	messageOut->gps_dr_aiding_status.channel_12 = messageIn.gps_dr_aiding_status.channel_12;
	messageOut->gps_dr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_dr_aiding_status.TimeBiasRepartitionEvent;
	messageOut->gps_dr_aiding_status.channel_1 = messageIn.gps_dr_aiding_status.channel_1;
	messageOut->gps_dr_aiding_status.channel_2 = messageIn.gps_dr_aiding_status.channel_2;
	messageOut->gps_dr_aiding_status.channel_3 = messageIn.gps_dr_aiding_status.channel_3;
	messageOut->gps_dr_aiding_status.channel_4 = messageIn.gps_dr_aiding_status.channel_4;
	messageOut->gps_dr_aiding_status.channel_5 = messageIn.gps_dr_aiding_status.channel_5;
	messageOut->gps_dr_aiding_status.channel_6 = messageIn.gps_dr_aiding_status.channel_6;
	messageOut->gps_dr_aiding_status.channel_7 = messageIn.gps_dr_aiding_status.channel_7;
	messageOut->gps_dr_aiding_status.channel_8 = messageIn.gps_dr_aiding_status.channel_8;
	messageOut->gps_dr_aiding_status.channel_9 = messageIn.gps_dr_aiding_status.channel_9;
	messageOut->gps_dr_aiding_status.channel_10 = messageIn.gps_dr_aiding_status.channel_10;
	messageOut->gps_dr_aiding_status.channel_11 = messageIn.gps_dr_aiding_status.channel_11;
	messageOut->gps_dr_aiding_status.channel_12 = messageIn.gps_dr_aiding_status.channel_12;
	messageOut->gps_dr_aiding_status.TimeBiasRepartitionEvent = messageIn.gps_dr_aiding_status.TimeBiasRepartitionEvent;

	messageOut->ta_aiding_status.position_ecef_X = messageIn.ta_aiding_status.position_ecef_X;
	messageOut->ta_aiding_status.position_ecef_Y = messageIn.ta_aiding_status.position_ecef_Y;
	messageOut->ta_aiding_status.position_ecef_Z = messageIn.ta_aiding_status.position_ecef_Z;
	messageOut->ta_aiding_status.velocity_ecef_X = messageIn.ta_aiding_status.velocity_ecef_X;
	messageOut->ta_aiding_status.velocity_ecef_Y = messageIn.ta_aiding_status.velocity_ecef_Y;
	messageOut->ta_aiding_status.velocity_ecef_Z = messageIn.ta_aiding_status.velocity_ecef_Z;
	messageOut->ta_aiding_status.attitude_roll = messageIn.ta_aiding_status.attitude_roll;
	messageOut->ta_aiding_status.attitude_pitch = messageIn.ta_aiding_status.attitude_pitch;
	messageOut->ta_aiding_status.attitude_heading = messageIn.ta_aiding_status.attitude_heading;
	messageOut->ta_aiding_status.position_ecef_X = messageIn.ta_aiding_status.position_ecef_X;
	messageOut->ta_aiding_status.position_ecef_Y = messageIn.ta_aiding_status.position_ecef_Y;
	messageOut->ta_aiding_status.position_ecef_Z = messageIn.ta_aiding_status.position_ecef_Z;
	messageOut->ta_aiding_status.velocity_ecef_X = messageIn.ta_aiding_status.velocity_ecef_X;
	messageOut->ta_aiding_status.velocity_ecef_Y = messageIn.ta_aiding_status.velocity_ecef_Y;
	messageOut->ta_aiding_status.velocity_ecef_Z = messageIn.ta_aiding_status.velocity_ecef_Z;
	messageOut->ta_aiding_status.attitude_roll = messageIn.ta_aiding_status.attitude_roll;
	messageOut->ta_aiding_status.attitude_pitch = messageIn.ta_aiding_status.attitude_pitch;
	messageOut->ta_aiding_status.attitude_heading = messageIn.ta_aiding_status.attitude_heading;

	messageOut->gnss_pvt_aiding_status.latitude = messageIn.gnss_pvt_aiding_status.latitude;
	messageOut->gnss_pvt_aiding_status.longitude = messageIn.gnss_pvt_aiding_status.longitude;
	messageOut->gnss_pvt_aiding_status.altitude = messageIn.gnss_pvt_aiding_status.altitude;
	messageOut->gnss_pvt_aiding_status.velocity_north = messageIn.gnss_pvt_aiding_status.velocity_north;
	messageOut->gnss_pvt_aiding_status.velocity_east = messageIn.gnss_pvt_aiding_status.velocity_east;
	messageOut->gnss_pvt_aiding_status.velocity_down = messageIn.gnss_pvt_aiding_status.velocity_down;
	messageOut->gnss_pvt_aiding_status.latitude = messageIn.gnss_pvt_aiding_status.latitude;
	messageOut->gnss_pvt_aiding_status.longitude = messageIn.gnss_pvt_aiding_status.longitude;
	messageOut->gnss_pvt_aiding_status.altitude = messageIn.gnss_pvt_aiding_status.altitude;
	messageOut->gnss_pvt_aiding_status.velocity_north = messageIn.gnss_pvt_aiding_status.velocity_north;
	messageOut->gnss_pvt_aiding_status.velocity_east = messageIn.gnss_pvt_aiding_status.velocity_east;
	messageOut->gnss_pvt_aiding_status.velocity_down = messageIn.gnss_pvt_aiding_status.velocity_down;

	messageOut->misc_aiding_status.zero_velocity_x = messageIn.misc_aiding_status.zero_velocity_x;
	messageOut->misc_aiding_status.zero_velocity_y = messageIn.misc_aiding_status.zero_velocity_y;
	messageOut->misc_aiding_status.zero_velocity_z = messageIn.misc_aiding_status.zero_velocity_z;
	messageOut->misc_aiding_status.magnetic_heading = messageIn.misc_aiding_status.magnetic_heading;
	messageOut->misc_aiding_status.zero_heading_change = messageIn.misc_aiding_status.zero_heading_change;
	messageOut->misc_aiding_status.barometric_altitude = messageIn.misc_aiding_status.barometric_altitude;
	messageOut->misc_aiding_status.zero_velocity_x = messageIn.misc_aiding_status.zero_velocity_x;
	messageOut->misc_aiding_status.zero_velocity_y = messageIn.misc_aiding_status.zero_velocity_y;
	messageOut->misc_aiding_status.zero_velocity_z = messageIn.misc_aiding_status.zero_velocity_z;
	messageOut->misc_aiding_status.magnetic_heading = messageIn.misc_aiding_status.magnetic_heading;
	messageOut->misc_aiding_status.zero_heading_change = messageIn.misc_aiding_status.zero_heading_change;
	messageOut->misc_aiding_status.barometric_altitude = messageIn.misc_aiding_status.barometric_altitude;
	messageOut->Available_Measurements = messageIn.Available_Measurements;
	messageOut->Successfully_Processed = messageIn.Successfully_Processed;
	messageOut->Failed_Processing = messageIn.Failed_Processing;
}

void Msg_2421_pub_callback(uint8_t * buffer)
{
	Msg_2421 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2421 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2421);
	ROS_DEBUG("Message 0x2421 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2421_pub_initialized == false){
		init_2421(getRosHandle());}
	// Publish the message
	Msg_2421_pub.publish(msgStruct_2421);
	return;
}
