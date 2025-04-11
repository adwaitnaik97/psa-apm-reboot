#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5108.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5108 msgStruct_5108;

bool Msg_5108_pub_initialized = false;

ros::Publisher Msg_5108_pub;
void init_5108(ros::NodeHandle * n){
	Msg_5108_pub = n->advertise<hg_nav_node::Msg_5108>(MSG_5108_PATH, 5);
	Msg_5108_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5108_PATH);
	return;
}

void stop_5108(void){
	Msg_5108_pub.shutdown();
	Msg_5108_pub_initialized = false;
	ROS_INFO("0x5108 stopped");
	return;
}

// Msg_5108 to Topic
void convert(Msg_5108 messageIn, hg_nav_node::Msg_5108 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->RxClkBias = messageIn.RxClkBias;
	messageOut->Datum = messageIn.Datum;
	messageOut->NumberOfSvs = messageIn.NumberOfSvs;
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
	messageOut->GeoidSeparation = messageIn.GeoidSeparation;
}

// Topic to Msg_5108
void convert(hg_nav_node::Msg_5108 messageIn, Msg_5108 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->RxClkBias = messageIn.RxClkBias;
	messageOut->Datum = messageIn.Datum;
	messageOut->NumberOfSvs = messageIn.NumberOfSvs;
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
	messageOut->GeoidSeparation = messageIn.GeoidSeparation;
}

void Msg_5108_pub_callback(uint8_t * buffer)
{
	Msg_5108 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5108 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5108);
	ROS_DEBUG("Message 0x5108 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5108_pub_initialized == false){
		init_5108(getRosHandle());}
	// Publish the message
	Msg_5108_pub.publish(msgStruct_5108);
	return;
}
