#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6403.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6403 msgStruct_6403;

bool Msg_6403_pub_initialized = false;

ros::Publisher Msg_6403_pub;
void init_6403(ros::NodeHandle * n){
	Msg_6403_pub = n->advertise<hg_nav_node::Msg_6403>(MSG_6403_PATH, 5);
	Msg_6403_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6403_PATH);
	return;
}

void stop_6403(void){
	Msg_6403_pub.shutdown();
	Msg_6403_pub_initialized = false;
	ROS_INFO("0x6403 stopped");
	return;
}

// Msg_6403 to Topic
void convert(Msg_6403 messageIn, hg_nav_node::Msg_6403 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;

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
	messageOut->LatitudeSTDV = messageIn.LatitudeSTDV;
	messageOut->LongitudeSTDV = messageIn.LongitudeSTDV;
	messageOut->AltitudeHeightAboveEllipsoidSTDV = messageIn.AltitudeHeightAboveEllipsoidSTDV;
}

// Topic to Msg_6403
void convert(hg_nav_node::Msg_6403 messageIn, Msg_6403 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;

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
	messageOut->LatitudeSTDV = messageIn.LatitudeSTDV;
	messageOut->LongitudeSTDV = messageIn.LongitudeSTDV;
	messageOut->AltitudeHeightAboveEllipsoidSTDV = messageIn.AltitudeHeightAboveEllipsoidSTDV;
}

void Msg_6403_pub_callback(uint8_t * buffer)
{
	Msg_6403 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6403 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6403);
	ROS_DEBUG("Message 0x6403 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6403_pub_initialized == false){
		init_6403(getRosHandle());}
	// Publish the message
	Msg_6403_pub.publish(msgStruct_6403);
	return;
}
