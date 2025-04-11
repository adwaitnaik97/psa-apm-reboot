#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6108.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6108 msgStruct_6108;

bool Msg_6108_pub_initialized = false;

ros::Publisher Msg_6108_pub;
void init_6108(ros::NodeHandle * n){
	Msg_6108_pub = n->advertise<hg_nav_node::Msg_6108>(MSG_6108_PATH, 5);
	Msg_6108_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6108_PATH);
	return;
}

void stop_6108(void){
	Msg_6108_pub.shutdown();
	Msg_6108_pub_initialized = false;
	ROS_INFO("0x6108 stopped");
	return;
}

// Msg_6108 to Topic
void convert(Msg_6108 messageIn, hg_nav_node::Msg_6108 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

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
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->RxClkBias = messageIn.RxClkBias;
	messageOut->Datum = messageIn.Datum;
	messageOut->TimeReference = messageIn.TimeReference;
	messageOut->NumberOfSvs = messageIn.NumberOfSvs;
	messageOut->RTKfixProg = messageIn.RTKfixProg;
	messageOut->CorrInfo = messageIn.CorrInfo;
	messageOut->SignalInfo = messageIn.SignalInfo;
	messageOut->PPPInfo = messageIn.PPPInfo;
	messageOut->LatitudeSTDV = messageIn.LatitudeSTDV;
	messageOut->LongitudeSTDV = messageIn.LongitudeSTDV;
	messageOut->AltitudeHeightAboveEllipsoidSTDV = messageIn.AltitudeHeightAboveEllipsoidSTDV;
	messageOut->NorthVelocitySTDV = messageIn.NorthVelocitySTDV;
	messageOut->EastVelocitySTDV = messageIn.EastVelocitySTDV;
	messageOut->DownVelocitySTDV = messageIn.DownVelocitySTDV;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->PDOP = messageIn.PDOP;
	messageOut->HDOP = messageIn.HDOP;
	messageOut->VDOP = messageIn.VDOP;
	messageOut->TDOP = messageIn.TDOP;
	messageOut->Undulation = messageIn.Undulation;
}

// Topic to Msg_6108
void convert(hg_nav_node::Msg_6108 messageIn, Msg_6108 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

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
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->RxClkBias = messageIn.RxClkBias;
	messageOut->Datum = messageIn.Datum;
	messageOut->TimeReference = messageIn.TimeReference;
	messageOut->NumberOfSvs = messageIn.NumberOfSvs;
	messageOut->RTKfixProg = messageIn.RTKfixProg;
	messageOut->CorrInfo = messageIn.CorrInfo;
	messageOut->SignalInfo = messageIn.SignalInfo;
	messageOut->PPPInfo = messageIn.PPPInfo;
	messageOut->LatitudeSTDV = messageIn.LatitudeSTDV;
	messageOut->LongitudeSTDV = messageIn.LongitudeSTDV;
	messageOut->AltitudeHeightAboveEllipsoidSTDV = messageIn.AltitudeHeightAboveEllipsoidSTDV;
	messageOut->NorthVelocitySTDV = messageIn.NorthVelocitySTDV;
	messageOut->EastVelocitySTDV = messageIn.EastVelocitySTDV;
	messageOut->DownVelocitySTDV = messageIn.DownVelocitySTDV;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->PDOP = messageIn.PDOP;
	messageOut->HDOP = messageIn.HDOP;
	messageOut->VDOP = messageIn.VDOP;
	messageOut->TDOP = messageIn.TDOP;
	messageOut->Undulation = messageIn.Undulation;
}

void Msg_6108_pub_callback(uint8_t * buffer)
{
	Msg_6108 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6108 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6108);
	ROS_DEBUG("Message 0x6108 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6108_pub_initialized == false){
		init_6108(getRosHandle());}
	// Publish the message
	Msg_6108_pub.publish(msgStruct_6108);
	return;
}
