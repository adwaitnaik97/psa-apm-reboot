#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2401.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2401 msgStruct_2401;

bool Msg_2401_pub_initialized = false;

ros::Publisher Msg_2401_pub;
void init_2401(ros::NodeHandle * n){
	Msg_2401_pub = n->advertise<hg_nav_node::Msg_2401>(MSG_2401_PATH, 5);
	Msg_2401_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2401_PATH);
	return;
}

void stop_2401(void){
	Msg_2401_pub.shutdown();
	Msg_2401_pub_initialized = false;
	ROS_INFO("0x2401 stopped");
	return;
}

// Msg_2401 to Topic
void convert(Msg_2401 messageIn, hg_nav_node::Msg_2401 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->InsGnssSummary = messageIn.InsGnssSummary;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->gps_figure_of_merit = messageIn.gps_figure_of_merit;
	messageOut->ins_blended_figure_of_merit = messageIn.ins_blended_figure_of_merit;
	messageOut->PositionTov = messageIn.PositionTov;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->AltitudeMeanSeaLevel = messageIn.AltitudeMeanSeaLevel;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->VelocityTov = messageIn.VelocityTov;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->EcefVelocityX = messageIn.EcefVelocityX;
	messageOut->EcefVelocityY = messageIn.EcefVelocityY;
	messageOut->EcefVelocityZ = messageIn.EcefVelocityZ;
	messageOut->AttitudeTov = messageIn.AttitudeTov;
	messageOut->EulerAnglesRoll = messageIn.EulerAnglesRoll;
	messageOut->EulerAnglesPitch = messageIn.EulerAnglesPitch;
	messageOut->EulerAnglesTrueHeading = messageIn.EulerAnglesTrueHeading;
	messageOut->wander_angle = messageIn.wander_angle;
	messageOut->DCM11 = messageIn.DCM11;
	messageOut->DCM12 = messageIn.DCM12;
	messageOut->DCM13 = messageIn.DCM13;
	messageOut->DCM21 = messageIn.DCM21;
	messageOut->DCM22 = messageIn.DCM22;
	messageOut->DCM23 = messageIn.DCM23;
	messageOut->DCM31 = messageIn.DCM31;
	messageOut->DCM32 = messageIn.DCM32;
	messageOut->DCM33 = messageIn.DCM33;
	messageOut->angular_rate_x = messageIn.angular_rate_x;
	messageOut->angular_rate_y = messageIn.angular_rate_y;
	messageOut->angular_rate_z = messageIn.angular_rate_z;
	messageOut->linear_acceleration_x = messageIn.linear_acceleration_x;
	messageOut->linear_acceleration_y = messageIn.linear_acceleration_y;
	messageOut->linear_acceleration_z = messageIn.linear_acceleration_z;
	messageOut->attitude_figure_of_merit = messageIn.attitude_figure_of_merit;
	messageOut->q0_vehicle_body_to_ecef = messageIn.q0_vehicle_body_to_ecef;
	messageOut->q1_vehicle_body_to_ecef = messageIn.q1_vehicle_body_to_ecef;
	messageOut->q2_vehicle_body_to_ecef = messageIn.q2_vehicle_body_to_ecef;
	messageOut->q3_vehicle_body_to_ecef = messageIn.q3_vehicle_body_to_ecef;
}

// Topic to Msg_2401
void convert(hg_nav_node::Msg_2401 messageIn, Msg_2401 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->InsGnssSummary = messageIn.InsGnssSummary;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->utc_time_figure_of_merit = messageIn.utc_time_figure_of_merit;
	messageOut->gps_figure_of_merit = messageIn.gps_figure_of_merit;
	messageOut->ins_blended_figure_of_merit = messageIn.ins_blended_figure_of_merit;
	messageOut->PositionTov = messageIn.PositionTov;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->AltitudeMeanSeaLevel = messageIn.AltitudeMeanSeaLevel;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->VelocityTov = messageIn.VelocityTov;
	messageOut->NorthVelocity = messageIn.NorthVelocity;
	messageOut->EastVelocity = messageIn.EastVelocity;
	messageOut->DownVelocity = messageIn.DownVelocity;
	messageOut->EcefVelocityX = messageIn.EcefVelocityX;
	messageOut->EcefVelocityY = messageIn.EcefVelocityY;
	messageOut->EcefVelocityZ = messageIn.EcefVelocityZ;
	messageOut->AttitudeTov = messageIn.AttitudeTov;
	messageOut->EulerAnglesRoll = messageIn.EulerAnglesRoll;
	messageOut->EulerAnglesPitch = messageIn.EulerAnglesPitch;
	messageOut->EulerAnglesTrueHeading = messageIn.EulerAnglesTrueHeading;
	messageOut->wander_angle = messageIn.wander_angle;
	messageOut->DCM11 = messageIn.DCM11;
	messageOut->DCM12 = messageIn.DCM12;
	messageOut->DCM13 = messageIn.DCM13;
	messageOut->DCM21 = messageIn.DCM21;
	messageOut->DCM22 = messageIn.DCM22;
	messageOut->DCM23 = messageIn.DCM23;
	messageOut->DCM31 = messageIn.DCM31;
	messageOut->DCM32 = messageIn.DCM32;
	messageOut->DCM33 = messageIn.DCM33;
	messageOut->angular_rate_x = messageIn.angular_rate_x;
	messageOut->angular_rate_y = messageIn.angular_rate_y;
	messageOut->angular_rate_z = messageIn.angular_rate_z;
	messageOut->linear_acceleration_x = messageIn.linear_acceleration_x;
	messageOut->linear_acceleration_y = messageIn.linear_acceleration_y;
	messageOut->linear_acceleration_z = messageIn.linear_acceleration_z;
	messageOut->attitude_figure_of_merit = messageIn.attitude_figure_of_merit;
	messageOut->q0_vehicle_body_to_ecef = messageIn.q0_vehicle_body_to_ecef;
	messageOut->q1_vehicle_body_to_ecef = messageIn.q1_vehicle_body_to_ecef;
	messageOut->q2_vehicle_body_to_ecef = messageIn.q2_vehicle_body_to_ecef;
	messageOut->q3_vehicle_body_to_ecef = messageIn.q3_vehicle_body_to_ecef;
}

void Msg_2401_pub_callback(uint8_t * buffer)
{
	Msg_2401 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2401 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2401);
	ROS_DEBUG("Message 0x2401 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2401_pub_initialized == false){
		init_2401(getRosHandle());}
	// Publish the message
	Msg_2401_pub.publish(msgStruct_2401);
	return;
}
