#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6112.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/init_sources_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6112 msgStruct_6112;

bool Msg_6112_pub_initialized = false;

ros::Publisher Msg_6112_pub;
void init_6112(ros::NodeHandle * n){
	Msg_6112_pub = n->advertise<hg_nav_node::Msg_6112>(MSG_6112_PATH, 5);
	Msg_6112_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6112_PATH);
	return;
}

void stop_6112(void){
	Msg_6112_pub.shutdown();
	Msg_6112_pub_initialized = false;
	ROS_INFO("0x6112 stopped");
	return;
}

// Msg_6112 to Topic
void convert(Msg_6112 messageIn, hg_nav_node::Msg_6112 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->Heading = messageIn.Heading;
	messageOut->Roll_stdv = messageIn.Roll_stdv;
	messageOut->Pitch_stdv = messageIn.Pitch_stdv;
	messageOut->Heading_stdv = messageIn.Heading_stdv;

	messageOut->Initialization_Sources.Position = messageIn.Initialization_Sources.Position;
	messageOut->Initialization_Sources.Velocity = messageIn.Initialization_Sources.Velocity;
	messageOut->Initialization_Sources.Attitude = messageIn.Initialization_Sources.Attitude;
	messageOut->Initialization_Sources.Heading = messageIn.Initialization_Sources.Heading;
	messageOut->Initialization_Sources.Position = messageIn.Initialization_Sources.Position;
	messageOut->Initialization_Sources.Velocity = messageIn.Initialization_Sources.Velocity;
	messageOut->Initialization_Sources.Attitude = messageIn.Initialization_Sources.Attitude;
	messageOut->Initialization_Sources.Heading = messageIn.Initialization_Sources.Heading;

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
	messageOut->Num_antennas = messageIn.Num_antennas;
}

// Topic to Msg_6112
void convert(hg_nav_node::Msg_6112 messageIn, Msg_6112 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Roll = messageIn.Roll;
	messageOut->Pitch = messageIn.Pitch;
	messageOut->Heading = messageIn.Heading;
	messageOut->Roll_stdv = messageIn.Roll_stdv;
	messageOut->Pitch_stdv = messageIn.Pitch_stdv;
	messageOut->Heading_stdv = messageIn.Heading_stdv;

	messageOut->Initialization_Sources.Position = messageIn.Initialization_Sources.Position;
	messageOut->Initialization_Sources.Velocity = messageIn.Initialization_Sources.Velocity;
	messageOut->Initialization_Sources.Attitude = messageIn.Initialization_Sources.Attitude;
	messageOut->Initialization_Sources.Heading = messageIn.Initialization_Sources.Heading;
	messageOut->Initialization_Sources.Position = messageIn.Initialization_Sources.Position;
	messageOut->Initialization_Sources.Velocity = messageIn.Initialization_Sources.Velocity;
	messageOut->Initialization_Sources.Attitude = messageIn.Initialization_Sources.Attitude;
	messageOut->Initialization_Sources.Heading = messageIn.Initialization_Sources.Heading;

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
	messageOut->Num_antennas = messageIn.Num_antennas;
}

void Msg_6112_pub_callback(uint8_t * buffer)
{
	Msg_6112 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6112 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6112);
	ROS_DEBUG("Message 0x6112 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6112_pub_initialized == false){
		init_6112(getRosHandle());}
	// Publish the message
	Msg_6112_pub.publish(msgStruct_6112);
	return;
}
