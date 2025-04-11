#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6111.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6111 msgStruct_6111;

bool Msg_6111_pub_initialized = false;

ros::Publisher Msg_6111_pub;
void init_6111(ros::NodeHandle * n){
	Msg_6111_pub = n->advertise<hg_nav_node::Msg_6111>(MSG_6111_PATH, 5);
	Msg_6111_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6111_PATH);
	return;
}

void stop_6111(void){
	Msg_6111_pub.shutdown();
	Msg_6111_pub_initialized = false;
	ROS_INFO("0x6111 stopped");
	return;
}

// Msg_6111 to Topic
void convert(Msg_6111 messageIn, hg_nav_node::Msg_6111 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;

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
	messageOut->Test1_RotationX = messageIn.Test1_RotationX;
	messageOut->Test1_RotationY = messageIn.Test1_RotationY;
	messageOut->Test1_RotationZ = messageIn.Test1_RotationZ;
	messageOut->Test1_RotationNormRate = messageIn.Test1_RotationNormRate;
	messageOut->Test1_RotationNormRateThreshold = messageIn.Test1_RotationNormRateThreshold;
	messageOut->Test2_SpeedValid = messageIn.Test2_SpeedValid;
	messageOut->Test2_Speed = messageIn.Test2_Speed;
	messageOut->Test2_SpeedThreshold = messageIn.Test2_SpeedThreshold;
	messageOut->Test3_AngularRateInstantX = messageIn.Test3_AngularRateInstantX;
	messageOut->Test3_AngularRateInstantY = messageIn.Test3_AngularRateInstantY;
	messageOut->Test3_AngularRateInstantZ = messageIn.Test3_AngularRateInstantZ;
	messageOut->Test3_InstantFilterBandwidth = messageIn.Test3_InstantFilterBandwidth;
	messageOut->Test3_AngularRateNominalX = messageIn.Test3_AngularRateNominalX;
	messageOut->Test3_AngularRateNominalY = messageIn.Test3_AngularRateNominalY;
	messageOut->Test3_AngularRateNominalZ = messageIn.Test3_AngularRateNominalZ;
	messageOut->Test3_NominalFilterBandwidth = messageIn.Test3_NominalFilterBandwidth;
	messageOut->Test3_AngularRateX = messageIn.Test3_AngularRateX;
	messageOut->Test3_AngularRateY = messageIn.Test3_AngularRateY;
	messageOut->Test3_AngularRateZ = messageIn.Test3_AngularRateZ;
	messageOut->Test3_AngularRateThreshold = messageIn.Test3_AngularRateThreshold;
	messageOut->Test4_LinearAcceleration = messageIn.Test4_LinearAcceleration;
	messageOut->Test4_LinearAccelerationThreshold = messageIn.Test4_LinearAccelerationThreshold;
	messageOut->Test5_OdometerDeltaDistance = messageIn.Test5_OdometerDeltaDistance;
	messageOut->Test5_OdometerDeltaDistanceThreshold = messageIn.Test5_OdometerDeltaDistanceThreshold;
	messageOut->SettlingTime_Odometer = messageIn.SettlingTime_Odometer;
	messageOut->SettlingTime_Stationary = messageIn.SettlingTime_Stationary;
	messageOut->SettlingTime_Threshold = messageIn.SettlingTime_Threshold;
}

// Topic to Msg_6111
void convert(hg_nav_node::Msg_6111 messageIn, Msg_6111 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;

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
	messageOut->Test1_RotationX = messageIn.Test1_RotationX;
	messageOut->Test1_RotationY = messageIn.Test1_RotationY;
	messageOut->Test1_RotationZ = messageIn.Test1_RotationZ;
	messageOut->Test1_RotationNormRate = messageIn.Test1_RotationNormRate;
	messageOut->Test1_RotationNormRateThreshold = messageIn.Test1_RotationNormRateThreshold;
	messageOut->Test2_SpeedValid = messageIn.Test2_SpeedValid;
	messageOut->Test2_Speed = messageIn.Test2_Speed;
	messageOut->Test2_SpeedThreshold = messageIn.Test2_SpeedThreshold;
	messageOut->Test3_AngularRateInstantX = messageIn.Test3_AngularRateInstantX;
	messageOut->Test3_AngularRateInstantY = messageIn.Test3_AngularRateInstantY;
	messageOut->Test3_AngularRateInstantZ = messageIn.Test3_AngularRateInstantZ;
	messageOut->Test3_InstantFilterBandwidth = messageIn.Test3_InstantFilterBandwidth;
	messageOut->Test3_AngularRateNominalX = messageIn.Test3_AngularRateNominalX;
	messageOut->Test3_AngularRateNominalY = messageIn.Test3_AngularRateNominalY;
	messageOut->Test3_AngularRateNominalZ = messageIn.Test3_AngularRateNominalZ;
	messageOut->Test3_NominalFilterBandwidth = messageIn.Test3_NominalFilterBandwidth;
	messageOut->Test3_AngularRateX = messageIn.Test3_AngularRateX;
	messageOut->Test3_AngularRateY = messageIn.Test3_AngularRateY;
	messageOut->Test3_AngularRateZ = messageIn.Test3_AngularRateZ;
	messageOut->Test3_AngularRateThreshold = messageIn.Test3_AngularRateThreshold;
	messageOut->Test4_LinearAcceleration = messageIn.Test4_LinearAcceleration;
	messageOut->Test4_LinearAccelerationThreshold = messageIn.Test4_LinearAccelerationThreshold;
	messageOut->Test5_OdometerDeltaDistance = messageIn.Test5_OdometerDeltaDistance;
	messageOut->Test5_OdometerDeltaDistanceThreshold = messageIn.Test5_OdometerDeltaDistanceThreshold;
	messageOut->SettlingTime_Odometer = messageIn.SettlingTime_Odometer;
	messageOut->SettlingTime_Stationary = messageIn.SettlingTime_Stationary;
	messageOut->SettlingTime_Threshold = messageIn.SettlingTime_Threshold;
}

void Msg_6111_pub_callback(uint8_t * buffer)
{
	Msg_6111 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6111 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6111);
	ROS_DEBUG("Message 0x6111 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6111_pub_initialized == false){
		init_6111(getRosHandle());}
	// Publish the message
	Msg_6111_pub.publish(msgStruct_6111);
	return;
}
