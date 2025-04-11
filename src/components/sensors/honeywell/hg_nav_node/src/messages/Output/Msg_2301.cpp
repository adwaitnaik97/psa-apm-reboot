#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2301.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_2301 msgStruct_2301;

bool Msg_2301_pub_initialized = false;

ros::Publisher Msg_2301_pub;
void init_2301(ros::NodeHandle * n){
	Msg_2301_pub = n->advertise<hg_nav_node::Msg_2301>(MSG_2301_PATH, 5);
	Msg_2301_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2301_PATH);
	return;
}

void stop_2301(void){
	Msg_2301_pub.shutdown();
	Msg_2301_pub_initialized = false;
	ROS_INFO("0x2301 stopped");
	return;
}

// Msg_2301 to Topic
void convert(Msg_2301 messageIn, hg_nav_node::Msg_2301 * messageOut)
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
	messageOut->angular_rate_x = messageIn.angular_rate_x;
	messageOut->angular_rate_y = messageIn.angular_rate_y;
	messageOut->angular_rate_z = messageIn.angular_rate_z;
	messageOut->linear_acceleration_x = messageIn.linear_acceleration_x;
	messageOut->linear_acceleration_y = messageIn.linear_acceleration_y;
	messageOut->linear_acceleration_z = messageIn.linear_acceleration_z;
	messageOut->outputFrame = messageIn.outputFrame;
}

// Topic to Msg_2301
void convert(hg_nav_node::Msg_2301 messageIn, Msg_2301 * messageOut)
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
	messageOut->angular_rate_x = messageIn.angular_rate_x;
	messageOut->angular_rate_y = messageIn.angular_rate_y;
	messageOut->angular_rate_z = messageIn.angular_rate_z;
	messageOut->linear_acceleration_x = messageIn.linear_acceleration_x;
	messageOut->linear_acceleration_y = messageIn.linear_acceleration_y;
	messageOut->linear_acceleration_z = messageIn.linear_acceleration_z;
	messageOut->outputFrame = messageIn.outputFrame;
}

void Msg_2301_pub_callback(uint8_t * buffer)
{
	Msg_2301 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2301 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2301);
	ROS_DEBUG("Message 0x2301 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2301_pub_initialized == false){
		init_2301(getRosHandle());}
	// Publish the message
	Msg_2301_pub.publish(msgStruct_2301);
	return;
}
