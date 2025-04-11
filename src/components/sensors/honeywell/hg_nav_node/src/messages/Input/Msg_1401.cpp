#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1401.h>
hg_nav_node::Msg_1401 msgStruct_1401;

ros::Subscriber Msg_1401_sub;
void init_1401(ros::NodeHandle * n){
	Msg_1401_sub = n->subscribe(MSG_1401_PATH, 5, Msg_1401_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1401_PATH);
	return;
}

void stop_1401(void){
	Msg_1401_sub.shutdown();
	ROS_INFO("0x1401 stopped");
	return;
}

// Msg_1401 to Topic
void convert(Msg_1401 messageIn, hg_nav_node::Msg_1401 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestACKNAKReply = messageIn.RequestACKNAKReply;
	messageOut->PositionTov = messageIn.PositionTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->PositionTimeReferenceMode = messageIn.PositionTimeReferenceMode;
	messageOut->PositionCoordinateFrame = messageIn.PositionCoordinateFrame;
	messageOut->PositionStdvValidity = messageIn.PositionStdvValidity;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->VelocityTov = messageIn.VelocityTov;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->VelocityTimeReferenceMode = messageIn.VelocityTimeReferenceMode;
	messageOut->VelocityCoordinateFrame = messageIn.VelocityCoordinateFrame;
	messageOut->VelocityStdvValidity = messageIn.VelocityStdvValidity;
	messageOut->VelocityNorthOrEcefVelocityX = messageIn.VelocityNorthOrEcefVelocityX;
	messageOut->VelocityEastOrEcefVelocityY = messageIn.VelocityEastOrEcefVelocityY;
	messageOut->VelocityDownOrEcefVelocityZ = messageIn.VelocityDownOrEcefVelocityZ;
	messageOut->AttitudeTov = messageIn.AttitudeTov;
	messageOut->AttitudeValidity = messageIn.AttitudeValidity;
	messageOut->AttitudeTimeReferenceMode = messageIn.AttitudeTimeReferenceMode;
	messageOut->AttitudeCoordinateFrame = messageIn.AttitudeCoordinateFrame;
	messageOut->AttitudeStdvValidity = messageIn.AttitudeStdvValidity;
	messageOut->EulerAnglesRoll = messageIn.EulerAnglesRoll;
	messageOut->DCM11 = messageIn.DCM11;
	messageOut->EulerAnglesPitch = messageIn.EulerAnglesPitch;
	messageOut->DCM12 = messageIn.DCM12;
	messageOut->EulerAnglesTrueHeading = messageIn.EulerAnglesTrueHeading;
	messageOut->DCM13 = messageIn.DCM13;
	messageOut->DCM21 = messageIn.DCM21;
	messageOut->DCM22 = messageIn.DCM22;
	messageOut->DCM23 = messageIn.DCM23;
	messageOut->DCM31 = messageIn.DCM31;
	messageOut->DCM32 = messageIn.DCM32;
	messageOut->DCM33 = messageIn.DCM33;
	messageOut->PositionStdvNorth = messageIn.PositionStdvNorth;
	messageOut->PositionStdvEast = messageIn.PositionStdvEast;
	messageOut->PositionStdvDown = messageIn.PositionStdvDown;
	messageOut->VelocityStdvNorth = messageIn.VelocityStdvNorth;
	messageOut->VelocityStdvEast = messageIn.VelocityStdvEast;
	messageOut->VelocityStdvDown = messageIn.VelocityStdvDown;
	messageOut->EulerAnglesStdvRoll = messageIn.EulerAnglesStdvRoll;
	messageOut->EulerAnglesStdvPitch = messageIn.EulerAnglesStdvPitch;
	messageOut->EulerAnglesStdvTrueHeading = messageIn.EulerAnglesStdvTrueHeading;
}

// Topic to Msg_1401
void convert(hg_nav_node::Msg_1401 messageIn, Msg_1401 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestACKNAKReply = messageIn.RequestACKNAKReply;
	messageOut->PositionTov = messageIn.PositionTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->PositionTimeReferenceMode = messageIn.PositionTimeReferenceMode;
	messageOut->PositionCoordinateFrame = messageIn.PositionCoordinateFrame;
	messageOut->PositionStdvValidity = messageIn.PositionStdvValidity;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->AltitudeHeightAboveEllipsoid = messageIn.AltitudeHeightAboveEllipsoid;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->VelocityTov = messageIn.VelocityTov;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->VelocityTimeReferenceMode = messageIn.VelocityTimeReferenceMode;
	messageOut->VelocityCoordinateFrame = messageIn.VelocityCoordinateFrame;
	messageOut->VelocityStdvValidity = messageIn.VelocityStdvValidity;
	messageOut->VelocityNorthOrEcefVelocityX = messageIn.VelocityNorthOrEcefVelocityX;
	messageOut->VelocityEastOrEcefVelocityY = messageIn.VelocityEastOrEcefVelocityY;
	messageOut->VelocityDownOrEcefVelocityZ = messageIn.VelocityDownOrEcefVelocityZ;
	messageOut->AttitudeTov = messageIn.AttitudeTov;
	messageOut->AttitudeValidity = messageIn.AttitudeValidity;
	messageOut->AttitudeTimeReferenceMode = messageIn.AttitudeTimeReferenceMode;
	messageOut->AttitudeCoordinateFrame = messageIn.AttitudeCoordinateFrame;
	messageOut->AttitudeStdvValidity = messageIn.AttitudeStdvValidity;
	messageOut->EulerAnglesRoll = messageIn.EulerAnglesRoll;
	messageOut->DCM11 = messageIn.DCM11;
	messageOut->EulerAnglesPitch = messageIn.EulerAnglesPitch;
	messageOut->DCM12 = messageIn.DCM12;
	messageOut->EulerAnglesTrueHeading = messageIn.EulerAnglesTrueHeading;
	messageOut->DCM13 = messageIn.DCM13;
	messageOut->DCM21 = messageIn.DCM21;
	messageOut->DCM22 = messageIn.DCM22;
	messageOut->DCM23 = messageIn.DCM23;
	messageOut->DCM31 = messageIn.DCM31;
	messageOut->DCM32 = messageIn.DCM32;
	messageOut->DCM33 = messageIn.DCM33;
	messageOut->PositionStdvNorth = messageIn.PositionStdvNorth;
	messageOut->PositionStdvEast = messageIn.PositionStdvEast;
	messageOut->PositionStdvDown = messageIn.PositionStdvDown;
	messageOut->VelocityStdvNorth = messageIn.VelocityStdvNorth;
	messageOut->VelocityStdvEast = messageIn.VelocityStdvEast;
	messageOut->VelocityStdvDown = messageIn.VelocityStdvDown;
	messageOut->EulerAnglesStdvRoll = messageIn.EulerAnglesStdvRoll;
	messageOut->EulerAnglesStdvPitch = messageIn.EulerAnglesStdvPitch;
	messageOut->EulerAnglesStdvTrueHeading = messageIn.EulerAnglesStdvTrueHeading;
}

void Msg_1401_sub_callback(const hg_nav_node::Msg_1401::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1401 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1401 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1401 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
