#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6428.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6428 msgStruct_6428;

bool Msg_6428_pub_initialized = false;

ros::Publisher Msg_6428_pub;
void init_6428(ros::NodeHandle * n){
	Msg_6428_pub = n->advertise<hg_nav_node::Msg_6428>(MSG_6428_PATH, 5);
	Msg_6428_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6428_PATH);
	return;
}

void stop_6428(void){
	Msg_6428_pub.shutdown();
	Msg_6428_pub_initialized = false;
	ROS_INFO("0x6428 stopped");
	return;
}

// Msg_6428 to Topic
void convert(Msg_6428 messageIn, hg_nav_node::Msg_6428 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->InsGnssSummary.INSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.INSMode);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.GPSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gps_pvt_pos_x = messageIn.gps_pvt_pos_x;
	messageOut->gps_pvt_pos_y = messageIn.gps_pvt_pos_y;
	messageOut->gps_pvt_pos_z = messageIn.gps_pvt_pos_z;
	messageOut->gps_pvt_vel_x = messageIn.gps_pvt_vel_x;
	messageOut->gps_pvt_vel_y = messageIn.gps_pvt_vel_y;
	messageOut->gps_pvt_vel_z = messageIn.gps_pvt_vel_z;
	messageOut->gnss_att_x = messageIn.gnss_att_x;
	messageOut->gnss_att_y = messageIn.gnss_att_y;
	messageOut->gnss_att_z = messageIn.gnss_att_z;
}

// Topic to Msg_6428
void convert(hg_nav_node::Msg_6428 messageIn, Msg_6428 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(messageIn.InsGnssSummary.INSMode.value);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(messageIn.InsGnssSummary.GPSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gps_pvt_pos_x = messageIn.gps_pvt_pos_x;
	messageOut->gps_pvt_pos_y = messageIn.gps_pvt_pos_y;
	messageOut->gps_pvt_pos_z = messageIn.gps_pvt_pos_z;
	messageOut->gps_pvt_vel_x = messageIn.gps_pvt_vel_x;
	messageOut->gps_pvt_vel_y = messageIn.gps_pvt_vel_y;
	messageOut->gps_pvt_vel_z = messageIn.gps_pvt_vel_z;
	messageOut->gnss_att_x = messageIn.gnss_att_x;
	messageOut->gnss_att_y = messageIn.gnss_att_y;
	messageOut->gnss_att_z = messageIn.gnss_att_z;
}

void Msg_6428_pub_callback(uint8_t * buffer)
{
	Msg_6428 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6428 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6428);
	ROS_DEBUG("Message 0x6428 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6428_pub_initialized == false){
		init_6428(getRosHandle());}
	// Publish the message
	Msg_6428_pub.publish(msgStruct_6428);
	return;
}
