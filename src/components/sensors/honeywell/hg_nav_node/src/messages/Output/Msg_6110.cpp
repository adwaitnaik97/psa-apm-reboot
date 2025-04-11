#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6110.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
#include <hg_nav_node/odo_body_vel_status_t.h>
hg_nav_node::Msg_6110 msgStruct_6110;

bool Msg_6110_pub_initialized = false;

ros::Publisher Msg_6110_pub;
void init_6110(ros::NodeHandle * n){
	Msg_6110_pub = n->advertise<hg_nav_node::Msg_6110>(MSG_6110_PATH, 5);
	Msg_6110_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6110_PATH);
	return;
}

void stop_6110(void){
	Msg_6110_pub.shutdown();
	Msg_6110_pub_initialized = false;
	ROS_INFO("0x6110 stopped");
	return;
}

// Msg_6110 to Topic
void convert(Msg_6110 messageIn, hg_nav_node::Msg_6110 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Odo_X_Body_Distance = messageIn.Odo_X_Body_Distance;
	messageOut->Odo_Y_Body_Distance = messageIn.Odo_Y_Body_Distance;
	messageOut->Odo_Z_Body_Distance = messageIn.Odo_Z_Body_Distance;
	messageOut->Odo_X_Body_Distance_Compensated = messageIn.Odo_X_Body_Distance_Compensated;
	messageOut->Odo_Y_Body_Distance_Compensated = messageIn.Odo_Y_Body_Distance_Compensated;
	messageOut->Odo_Z_Body_Distance_Compensated = messageIn.Odo_Z_Body_Distance_Compensated;

	messageOut->Odo_status.Velocity_Valid = messageIn.Odo_status.Velocity_Valid;
	messageOut->Odo_status.Odo_Pulse_Valid = messageIn.Odo_status.Odo_Pulse_Valid;
	messageOut->Odo_status.TOV_Mode = messageIn.Odo_status.TOV_Mode;
	messageOut->Odo_status.Vel_Sending_Unit_Status = messageIn.Odo_status.Vel_Sending_Unit_Status;
	messageOut->Odo_status.Zupt_Requested = messageIn.Odo_status.Zupt_Requested;
	messageOut->uart_latency = messageIn.uart_latency;
	messageOut->customer_latency = messageIn.customer_latency;

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
}

// Topic to Msg_6110
void convert(hg_nav_node::Msg_6110 messageIn, Msg_6110 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->Odo_X_Body_Distance = messageIn.Odo_X_Body_Distance;
	messageOut->Odo_Y_Body_Distance = messageIn.Odo_Y_Body_Distance;
	messageOut->Odo_Z_Body_Distance = messageIn.Odo_Z_Body_Distance;
	messageOut->Odo_X_Body_Distance_Compensated = messageIn.Odo_X_Body_Distance_Compensated;
	messageOut->Odo_Y_Body_Distance_Compensated = messageIn.Odo_Y_Body_Distance_Compensated;
	messageOut->Odo_Z_Body_Distance_Compensated = messageIn.Odo_Z_Body_Distance_Compensated;

	messageOut->Odo_status.Velocity_Valid = messageIn.Odo_status.Velocity_Valid;
	messageOut->Odo_status.Odo_Pulse_Valid = messageIn.Odo_status.Odo_Pulse_Valid;
	messageOut->Odo_status.TOV_Mode = messageIn.Odo_status.TOV_Mode;
	messageOut->Odo_status.Vel_Sending_Unit_Status = messageIn.Odo_status.Vel_Sending_Unit_Status;
	messageOut->Odo_status.Zupt_Requested = messageIn.Odo_status.Zupt_Requested;
	messageOut->uart_latency = messageIn.uart_latency;
	messageOut->customer_latency = messageIn.customer_latency;

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
}

void Msg_6110_pub_callback(uint8_t * buffer)
{
	Msg_6110 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6110 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6110);
	ROS_DEBUG("Message 0x6110 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6110_pub_initialized == false){
		init_6110(getRosHandle());}
	// Publish the message
	Msg_6110_pub.publish(msgStruct_6110);
	return;
}
