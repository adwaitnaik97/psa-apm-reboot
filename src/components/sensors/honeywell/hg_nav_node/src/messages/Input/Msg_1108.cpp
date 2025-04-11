#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1108.h>
hg_nav_node::Msg_1108 msgStruct_1108;

ros::Subscriber Msg_1108_sub;
void init_1108(ros::NodeHandle * n){
	Msg_1108_sub = n->subscribe(MSG_1108_PATH, 5, Msg_1108_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1108_PATH);
	return;
}

void stop_1108(void){
	Msg_1108_sub.shutdown();
	ROS_INFO("0x1108 stopped");
	return;
}

// Msg_1108 to Topic
void convert(Msg_1108 messageIn, hg_nav_node::Msg_1108 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestAckNak = messageIn.RequestAckNak;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->UnfilteredPointSolution = messageIn.UnfilteredPointSolution;
	messageOut->GnssMode = messageIn.GnssMode;
	messageOut->gpsPvtTov = messageIn.gpsPvtTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->PositionCoordinateFrame = messageIn.PositionCoordinateFrame;
	messageOut->MslGeoidValidity = messageIn.MslGeoidValidity;
	messageOut->MslGeoidType = messageIn.MslGeoidType;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->Altitude = messageIn.Altitude;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->MslGeoidAltitudeOrGeoidHeight = messageIn.MslGeoidAltitudeOrGeoidHeight;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->VelocityCoordinateFrame = messageIn.VelocityCoordinateFrame;
	messageOut->VelocityNorthOrEcefVelocityX = messageIn.VelocityNorthOrEcefVelocityX;
	messageOut->VelocityEastOrEcefVelocityY = messageIn.VelocityEastOrEcefVelocityY;
	messageOut->VelocityDownOrEcefVelocityZ = messageIn.VelocityDownOrEcefVelocityZ;
	messageOut->LeapSecondValidity = messageIn.LeapSecondValidity;
	messageOut->ReceiverClockBiasErrorValidity = messageIn.ReceiverClockBiasErrorValidity;
	messageOut->DopDataValidity = messageIn.DopDataValidity;
	messageOut->ConstellationChangeIndicator = messageIn.ConstellationChangeIndicator;
	messageOut->EstimatedPositionErrorValidity = messageIn.EstimatedPositionErrorValidity;
	messageOut->EstimatedVelocityErrorValidity = messageIn.EstimatedVelocityErrorValidity;
	messageOut->LeapSeconds = messageIn.LeapSeconds;
	messageOut->ReceiverClockBiasError = messageIn.ReceiverClockBiasError;
	messageOut->Tdop = messageIn.Tdop;
	messageOut->Vdop = messageIn.Vdop;
	messageOut->Hdop = messageIn.Hdop;
	messageOut->Pdop = messageIn.Pdop;
	messageOut->EstimatedVerticalPositionError = messageIn.EstimatedVerticalPositionError;
	messageOut->EstimatedHorizontalPositionError = messageIn.EstimatedHorizontalPositionError;
	messageOut->EstimatedVerticalVelocityError = messageIn.EstimatedVerticalVelocityError;
	messageOut->EstimatedHorizontalVelocityError = messageIn.EstimatedHorizontalVelocityError;
	messageOut->Fom = messageIn.Fom;
	messageOut->Tfom = messageIn.Tfom;
}

// Topic to Msg_1108
void convert(hg_nav_node::Msg_1108 messageIn, Msg_1108 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->RequestAckNak = messageIn.RequestAckNak;
	messageOut->TimeReferenceMode = messageIn.TimeReferenceMode;
	messageOut->UnfilteredPointSolution = messageIn.UnfilteredPointSolution;
	messageOut->GnssMode = messageIn.GnssMode;
	messageOut->gpsPvtTov = messageIn.gpsPvtTov;
	messageOut->PositionValidity = messageIn.PositionValidity;
	messageOut->PositionCoordinateFrame = messageIn.PositionCoordinateFrame;
	messageOut->MslGeoidValidity = messageIn.MslGeoidValidity;
	messageOut->MslGeoidType = messageIn.MslGeoidType;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->EcefPositionX = messageIn.EcefPositionX;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->EcefPositionY = messageIn.EcefPositionY;
	messageOut->Altitude = messageIn.Altitude;
	messageOut->EcefPositionZ = messageIn.EcefPositionZ;
	messageOut->MslGeoidAltitudeOrGeoidHeight = messageIn.MslGeoidAltitudeOrGeoidHeight;
	messageOut->VelocityValidity = messageIn.VelocityValidity;
	messageOut->VelocityCoordinateFrame = messageIn.VelocityCoordinateFrame;
	messageOut->VelocityNorthOrEcefVelocityX = messageIn.VelocityNorthOrEcefVelocityX;
	messageOut->VelocityEastOrEcefVelocityY = messageIn.VelocityEastOrEcefVelocityY;
	messageOut->VelocityDownOrEcefVelocityZ = messageIn.VelocityDownOrEcefVelocityZ;
	messageOut->LeapSecondValidity = messageIn.LeapSecondValidity;
	messageOut->ReceiverClockBiasErrorValidity = messageIn.ReceiverClockBiasErrorValidity;
	messageOut->DopDataValidity = messageIn.DopDataValidity;
	messageOut->ConstellationChangeIndicator = messageIn.ConstellationChangeIndicator;
	messageOut->EstimatedPositionErrorValidity = messageIn.EstimatedPositionErrorValidity;
	messageOut->EstimatedVelocityErrorValidity = messageIn.EstimatedVelocityErrorValidity;
	messageOut->LeapSeconds = messageIn.LeapSeconds;
	messageOut->ReceiverClockBiasError = messageIn.ReceiverClockBiasError;
	messageOut->Tdop = messageIn.Tdop;
	messageOut->Vdop = messageIn.Vdop;
	messageOut->Hdop = messageIn.Hdop;
	messageOut->Pdop = messageIn.Pdop;
	messageOut->EstimatedVerticalPositionError = messageIn.EstimatedVerticalPositionError;
	messageOut->EstimatedHorizontalPositionError = messageIn.EstimatedHorizontalPositionError;
	messageOut->EstimatedVerticalVelocityError = messageIn.EstimatedVerticalVelocityError;
	messageOut->EstimatedHorizontalVelocityError = messageIn.EstimatedHorizontalVelocityError;
	messageOut->Fom = messageIn.Fom;
	messageOut->Tfom = messageIn.Tfom;
}

void Msg_1108_sub_callback(const hg_nav_node::Msg_1108::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1108 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1108 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1108 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
