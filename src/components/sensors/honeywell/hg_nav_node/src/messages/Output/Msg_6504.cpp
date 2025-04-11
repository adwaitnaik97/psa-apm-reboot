#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6504.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6504 msgStruct_6504;

bool Msg_6504_pub_initialized = false;

ros::Publisher Msg_6504_pub;
void init_6504(ros::NodeHandle * n){
	Msg_6504_pub = n->advertise<hg_nav_node::Msg_6504>(MSG_6504_PATH, 5);
	Msg_6504_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6504_PATH);
	return;
}

void stop_6504(void){
	Msg_6504_pub.shutdown();
	Msg_6504_pub_initialized = false;
	ROS_INFO("0x6504 stopped");
	return;
}

// Msg_6504 to Topic
void convert(Msg_6504 messageIn, hg_nav_node::Msg_6504 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;

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
	messageOut->NorthVelocitySTDV = messageIn.NorthVelocitySTDV;
	messageOut->EastVelocitySTDV = messageIn.EastVelocitySTDV;
	messageOut->DownVelocitySTDV = messageIn.DownVelocitySTDV;
}

// Topic to Msg_6504
void convert(hg_nav_node::Msg_6504 messageIn, Msg_6504 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;

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
	messageOut->NorthVelocitySTDV = messageIn.NorthVelocitySTDV;
	messageOut->EastVelocitySTDV = messageIn.EastVelocitySTDV;
	messageOut->DownVelocitySTDV = messageIn.DownVelocitySTDV;
}

void Msg_6504_pub_callback(uint8_t * buffer)
{
	Msg_6504 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6504 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6504);
	ROS_DEBUG("Message 0x6504 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6504_pub_initialized == false){
		init_6504(getRosHandle());}
	// Publish the message
	Msg_6504_pub.publish(msgStruct_6504);
	return;
}
