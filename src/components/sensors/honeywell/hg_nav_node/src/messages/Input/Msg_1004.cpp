#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1004.h>
hg_nav_node::Msg_1004 msgStruct_1004;

ros::Subscriber Msg_1004_sub;
void init_1004(ros::NodeHandle * n){
	Msg_1004_sub = n->subscribe(MSG_1004_PATH, 5, Msg_1004_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1004_PATH);
	return;
}

void stop_1004(void){
	Msg_1004_sub.shutdown();
	ROS_INFO("0x1004 stopped");
	return;
}

// Msg_1004 to Topic
void convert(Msg_1004 messageIn, hg_nav_node::Msg_1004 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->ULV_GPSSigLatencyL1 = messageIn.ULV_GPSSigLatencyL1;
	messageOut->ULV_GPSSigLatencyL2 = messageIn.ULV_GPSSigLatencyL2;
	messageOut->ULV_VehFrametoCaseFrame = messageIn.ULV_VehFrametoCaseFrame;
	messageOut->ULV_Vehicle_Case_FrameLeverArm = messageIn.ULV_Vehicle_Case_FrameLeverArm;
	messageOut->ULV_LeverArmToPort4GPSAnt = messageIn.ULV_LeverArmToPort4GPSAnt;
	messageOut->ULV_MeasFrametoVehFrame = messageIn.ULV_MeasFrametoVehFrame;
	messageOut->ULV_LeverArmToICDNav = messageIn.ULV_LeverArmToICDNav;
	messageOut->ULV_LeverArmToICDGPSAntenna = messageIn.ULV_LeverArmToICDGPSAntenna;
	messageOut->ULV_LocationOfAutopilotFCData = messageIn.ULV_LocationOfAutopilotFCData;
	messageOut->L1SignalLatency = messageIn.L1SignalLatency;
	messageOut->L2SignalLatency = messageIn.L2SignalLatency;
	messageOut->Vehicle_to_Case_Roll = messageIn.Vehicle_to_Case_Roll;
	messageOut->Vehicle_to_Case_C11 = messageIn.Vehicle_to_Case_C11;
	messageOut->Vehicle_to_Case_Pitch = messageIn.Vehicle_to_Case_Pitch;
	messageOut->Vehicle_to_Case_C12 = messageIn.Vehicle_to_Case_C12;
	messageOut->Vehicle_to_Case_Heading = messageIn.Vehicle_to_Case_Heading;
	messageOut->Vehicle_to_Case_C13 = messageIn.Vehicle_to_Case_C13;
	messageOut->Vehicle_to_Case_C21 = messageIn.Vehicle_to_Case_C21;
	messageOut->Vehicle_to_Case_C22 = messageIn.Vehicle_to_Case_C22;
	messageOut->Vehicle_to_Case_C23 = messageIn.Vehicle_to_Case_C23;
	messageOut->Vehicle_to_Case_C31 = messageIn.Vehicle_to_Case_C31;
	messageOut->Vehicle_to_Case_C32 = messageIn.Vehicle_to_Case_C32;
	messageOut->Vehicle_to_Case_C33 = messageIn.Vehicle_to_Case_C33;
	messageOut->VehCaseLvrArm_x = messageIn.VehCaseLvrArm_x;
	messageOut->VehCaseLvrArm_y = messageIn.VehCaseLvrArm_y;
	messageOut->VehCaseLvrArm_z = messageIn.VehCaseLvrArm_z;
	messageOut->VehGPSPort4LvrArm_x = messageIn.VehGPSPort4LvrArm_x;
	messageOut->VehGPSPort4LvrArm_y = messageIn.VehGPSPort4LvrArm_y;
	messageOut->VehGPSPort4LvrArm_z = messageIn.VehGPSPort4LvrArm_z;
	messageOut->Meas_to_Vehicle_Roll = messageIn.Meas_to_Vehicle_Roll;
	messageOut->Meas_to_Vehicle_C11 = messageIn.Meas_to_Vehicle_C11;
	messageOut->Meas_to_Vehicle_Pitch = messageIn.Meas_to_Vehicle_Pitch;
	messageOut->Meas_to_Vehicle_C12 = messageIn.Meas_to_Vehicle_C12;
	messageOut->Meas_to_Vehicle_Heading = messageIn.Meas_to_Vehicle_Heading;
	messageOut->Meas_to_Vehicle_C13 = messageIn.Meas_to_Vehicle_C13;
	messageOut->Meas_to_Vehicle_C21 = messageIn.Meas_to_Vehicle_C21;
	messageOut->Meas_to_Vehicle_C22 = messageIn.Meas_to_Vehicle_C22;
	messageOut->Meas_to_Vehicle_C23 = messageIn.Meas_to_Vehicle_C23;
	messageOut->Meas_to_Vehicle_C31 = messageIn.Meas_to_Vehicle_C31;
	messageOut->Meas_to_Vehicle_C32 = messageIn.Meas_to_Vehicle_C32;
	messageOut->Meas_to_Vehicle_C33 = messageIn.Meas_to_Vehicle_C33;
	messageOut->VehICDNavLvrArm_x = messageIn.VehICDNavLvrArm_x;
	messageOut->VehICDNavLvrArm_y = messageIn.VehICDNavLvrArm_y;
	messageOut->VehICDNavLvrArm_z = messageIn.VehICDNavLvrArm_z;
	messageOut->VehGPSPort4LvrArm_x_2 = messageIn.VehGPSPort4LvrArm_x_2;
	messageOut->VehGPSPort4LvrArm_y_2 = messageIn.VehGPSPort4LvrArm_y_2;
	messageOut->VehGPSPort4LvrArm_z_2 = messageIn.VehGPSPort4LvrArm_z_2;
}

// Topic to Msg_1004
void convert(hg_nav_node::Msg_1004 messageIn, Msg_1004 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->ULV_GPSSigLatencyL1 = messageIn.ULV_GPSSigLatencyL1;
	messageOut->ULV_GPSSigLatencyL2 = messageIn.ULV_GPSSigLatencyL2;
	messageOut->ULV_VehFrametoCaseFrame = messageIn.ULV_VehFrametoCaseFrame;
	messageOut->ULV_Vehicle_Case_FrameLeverArm = messageIn.ULV_Vehicle_Case_FrameLeverArm;
	messageOut->ULV_LeverArmToPort4GPSAnt = messageIn.ULV_LeverArmToPort4GPSAnt;
	messageOut->ULV_MeasFrametoVehFrame = messageIn.ULV_MeasFrametoVehFrame;
	messageOut->ULV_LeverArmToICDNav = messageIn.ULV_LeverArmToICDNav;
	messageOut->ULV_LeverArmToICDGPSAntenna = messageIn.ULV_LeverArmToICDGPSAntenna;
	messageOut->ULV_LocationOfAutopilotFCData = messageIn.ULV_LocationOfAutopilotFCData;
	messageOut->L1SignalLatency = messageIn.L1SignalLatency;
	messageOut->L2SignalLatency = messageIn.L2SignalLatency;
	messageOut->Vehicle_to_Case_Roll = messageIn.Vehicle_to_Case_Roll;
	messageOut->Vehicle_to_Case_C11 = messageIn.Vehicle_to_Case_C11;
	messageOut->Vehicle_to_Case_Pitch = messageIn.Vehicle_to_Case_Pitch;
	messageOut->Vehicle_to_Case_C12 = messageIn.Vehicle_to_Case_C12;
	messageOut->Vehicle_to_Case_Heading = messageIn.Vehicle_to_Case_Heading;
	messageOut->Vehicle_to_Case_C13 = messageIn.Vehicle_to_Case_C13;
	messageOut->Vehicle_to_Case_C21 = messageIn.Vehicle_to_Case_C21;
	messageOut->Vehicle_to_Case_C22 = messageIn.Vehicle_to_Case_C22;
	messageOut->Vehicle_to_Case_C23 = messageIn.Vehicle_to_Case_C23;
	messageOut->Vehicle_to_Case_C31 = messageIn.Vehicle_to_Case_C31;
	messageOut->Vehicle_to_Case_C32 = messageIn.Vehicle_to_Case_C32;
	messageOut->Vehicle_to_Case_C33 = messageIn.Vehicle_to_Case_C33;
	messageOut->VehCaseLvrArm_x = messageIn.VehCaseLvrArm_x;
	messageOut->VehCaseLvrArm_y = messageIn.VehCaseLvrArm_y;
	messageOut->VehCaseLvrArm_z = messageIn.VehCaseLvrArm_z;
	messageOut->VehGPSPort4LvrArm_x = messageIn.VehGPSPort4LvrArm_x;
	messageOut->VehGPSPort4LvrArm_y = messageIn.VehGPSPort4LvrArm_y;
	messageOut->VehGPSPort4LvrArm_z = messageIn.VehGPSPort4LvrArm_z;
	messageOut->Meas_to_Vehicle_Roll = messageIn.Meas_to_Vehicle_Roll;
	messageOut->Meas_to_Vehicle_C11 = messageIn.Meas_to_Vehicle_C11;
	messageOut->Meas_to_Vehicle_Pitch = messageIn.Meas_to_Vehicle_Pitch;
	messageOut->Meas_to_Vehicle_C12 = messageIn.Meas_to_Vehicle_C12;
	messageOut->Meas_to_Vehicle_Heading = messageIn.Meas_to_Vehicle_Heading;
	messageOut->Meas_to_Vehicle_C13 = messageIn.Meas_to_Vehicle_C13;
	messageOut->Meas_to_Vehicle_C21 = messageIn.Meas_to_Vehicle_C21;
	messageOut->Meas_to_Vehicle_C22 = messageIn.Meas_to_Vehicle_C22;
	messageOut->Meas_to_Vehicle_C23 = messageIn.Meas_to_Vehicle_C23;
	messageOut->Meas_to_Vehicle_C31 = messageIn.Meas_to_Vehicle_C31;
	messageOut->Meas_to_Vehicle_C32 = messageIn.Meas_to_Vehicle_C32;
	messageOut->Meas_to_Vehicle_C33 = messageIn.Meas_to_Vehicle_C33;
	messageOut->VehICDNavLvrArm_x = messageIn.VehICDNavLvrArm_x;
	messageOut->VehICDNavLvrArm_y = messageIn.VehICDNavLvrArm_y;
	messageOut->VehICDNavLvrArm_z = messageIn.VehICDNavLvrArm_z;
	messageOut->VehGPSPort4LvrArm_x_2 = messageIn.VehGPSPort4LvrArm_x_2;
	messageOut->VehGPSPort4LvrArm_y_2 = messageIn.VehGPSPort4LvrArm_y_2;
	messageOut->VehGPSPort4LvrArm_z_2 = messageIn.VehGPSPort4LvrArm_z_2;
}

void Msg_1004_sub_callback(const hg_nav_node::Msg_1004::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1004 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1004 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1004 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
