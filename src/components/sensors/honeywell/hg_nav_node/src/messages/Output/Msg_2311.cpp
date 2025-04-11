#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2311.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_2311 msgStruct_2311;

bool Msg_2311_pub_initialized = false;

ros::Publisher Msg_2311_pub;
void init_2311(ros::NodeHandle * n){
	Msg_2311_pub = n->advertise<hg_nav_node::Msg_2311>(MSG_2311_PATH, 5);
	Msg_2311_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2311_PATH);
	return;
}

void stop_2311(void){
	Msg_2311_pub.shutdown();
	Msg_2311_pub_initialized = false;
	ROS_INFO("0x2311 stopped");
	return;
}

// Msg_2311 to Topic
void convert(Msg_2311 messageIn, hg_nav_node::Msg_2311 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;

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
	messageOut->delta_theta_x = messageIn.delta_theta_x;
	messageOut->delta_theta_y = messageIn.delta_theta_y;
	messageOut->delta_theta_z = messageIn.delta_theta_z;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->delta_velocity_x = messageIn.delta_velocity_x;
	messageOut->delta_velocity_y = messageIn.delta_velocity_y;
	messageOut->delta_velocity_z = messageIn.delta_velocity_z;
	messageOut->gpsTov = messageIn.gpsTov;
}

// Topic to Msg_2311
void convert(hg_nav_node::Msg_2311 messageIn, Msg_2311 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;

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
	messageOut->delta_theta_x = messageIn.delta_theta_x;
	messageOut->delta_theta_y = messageIn.delta_theta_y;
	messageOut->delta_theta_z = messageIn.delta_theta_z;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->delta_velocity_x = messageIn.delta_velocity_x;
	messageOut->delta_velocity_y = messageIn.delta_velocity_y;
	messageOut->delta_velocity_z = messageIn.delta_velocity_z;
	messageOut->gpsTov = messageIn.gpsTov;
}

void Msg_2311_pub_callback(uint8_t * buffer)
{
	Msg_2311 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2311 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2311);
	ROS_DEBUG("Message 0x2311 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2311_pub_initialized == false){
		init_2311(getRosHandle());}
	// Publish the message
	Msg_2311_pub.publish(msgStruct_2311);
	return;
}
