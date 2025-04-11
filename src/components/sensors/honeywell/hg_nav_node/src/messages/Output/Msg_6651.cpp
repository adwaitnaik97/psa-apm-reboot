#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6651.h>
#include <hg_nav_node/data_status_t.h>
#include <hg_nav_node/port_type_t.h>
#include <hg_nav_node/threshold_status_t.h>
hg_nav_node::Msg_6651 msgStruct_6651;

bool Msg_6651_pub_initialized = false;

ros::Publisher Msg_6651_pub;
void init_6651(ros::NodeHandle * n){
	Msg_6651_pub = n->advertise<hg_nav_node::Msg_6651>(MSG_6651_PATH, 5);
	Msg_6651_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6651_PATH);
	return;
}

void stop_6651(void){
	Msg_6651_pub.shutdown();
	Msg_6651_pub_initialized = false;
	ROS_INFO("0x6651 stopped");
	return;
}

// Msg_6651 to Topic
void convert(Msg_6651 messageIn, hg_nav_node::Msg_6651 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->ImuDataStatus.portNum = messageIn.ImuDataStatus.portNum;
	messageOut->ImuDataStatus.portType.value = static_cast<uint8_t>(messageIn.ImuDataStatus.portType);
	messageOut->ImuDataStatus.active = messageIn.ImuDataStatus.active;
	messageOut->ImuDataStatus.portNum = messageIn.ImuDataStatus.portNum;
	messageOut->ImuDataStatus.portType.value = static_cast<uint8_t>(messageIn.ImuDataStatus.portType);
	messageOut->ImuDataStatus.active = messageIn.ImuDataStatus.active;

	messageOut->GnssDatatStatus.portNum = messageIn.GnssDatatStatus.portNum;
	messageOut->GnssDatatStatus.portType.value = static_cast<uint8_t>(messageIn.GnssDatatStatus.portType);
	messageOut->GnssDatatStatus.active = messageIn.GnssDatatStatus.active;
	messageOut->GnssDatatStatus.portNum = messageIn.GnssDatatStatus.portNum;
	messageOut->GnssDatatStatus.portType.value = static_cast<uint8_t>(messageIn.GnssDatatStatus.portType);
	messageOut->GnssDatatStatus.active = messageIn.GnssDatatStatus.active;

	messageOut->PositionStatus.x_axis = messageIn.PositionStatus.x_axis;
	messageOut->PositionStatus.y_axis = messageIn.PositionStatus.y_axis;
	messageOut->PositionStatus.z_axis = messageIn.PositionStatus.z_axis;
	messageOut->PositionStatus.enabled = messageIn.PositionStatus.enabled;
	messageOut->PositionStatus.x_axis = messageIn.PositionStatus.x_axis;
	messageOut->PositionStatus.y_axis = messageIn.PositionStatus.y_axis;
	messageOut->PositionStatus.z_axis = messageIn.PositionStatus.z_axis;
	messageOut->PositionStatus.enabled = messageIn.PositionStatus.enabled;

	messageOut->VelocityStatus.x_axis = messageIn.VelocityStatus.x_axis;
	messageOut->VelocityStatus.y_axis = messageIn.VelocityStatus.y_axis;
	messageOut->VelocityStatus.z_axis = messageIn.VelocityStatus.z_axis;
	messageOut->VelocityStatus.enabled = messageIn.VelocityStatus.enabled;
	messageOut->VelocityStatus.x_axis = messageIn.VelocityStatus.x_axis;
	messageOut->VelocityStatus.y_axis = messageIn.VelocityStatus.y_axis;
	messageOut->VelocityStatus.z_axis = messageIn.VelocityStatus.z_axis;
	messageOut->VelocityStatus.enabled = messageIn.VelocityStatus.enabled;

	messageOut->AttitudeStatus.x_axis = messageIn.AttitudeStatus.x_axis;
	messageOut->AttitudeStatus.y_axis = messageIn.AttitudeStatus.y_axis;
	messageOut->AttitudeStatus.z_axis = messageIn.AttitudeStatus.z_axis;
	messageOut->AttitudeStatus.enabled = messageIn.AttitudeStatus.enabled;
	messageOut->AttitudeStatus.x_axis = messageIn.AttitudeStatus.x_axis;
	messageOut->AttitudeStatus.y_axis = messageIn.AttitudeStatus.y_axis;
	messageOut->AttitudeStatus.z_axis = messageIn.AttitudeStatus.z_axis;
	messageOut->AttitudeStatus.enabled = messageIn.AttitudeStatus.enabled;
	messageOut->NavAxesSelected = messageIn.NavAxesSelected;
}

// Topic to Msg_6651
void convert(hg_nav_node::Msg_6651 messageIn, Msg_6651 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->ImuDataStatus.portNum = messageIn.ImuDataStatus.portNum;
	messageOut->ImuDataStatus.portType = static_cast<port_type_t>(messageIn.ImuDataStatus.portType.value);
	messageOut->ImuDataStatus.active = messageIn.ImuDataStatus.active;
	messageOut->ImuDataStatus.portNum = messageIn.ImuDataStatus.portNum;
	messageOut->ImuDataStatus.portType = static_cast<port_type_t>(messageIn.ImuDataStatus.portType.value);
	messageOut->ImuDataStatus.active = messageIn.ImuDataStatus.active;

	messageOut->GnssDatatStatus.portNum = messageIn.GnssDatatStatus.portNum;
	messageOut->GnssDatatStatus.portType = static_cast<port_type_t>(messageIn.GnssDatatStatus.portType.value);
	messageOut->GnssDatatStatus.active = messageIn.GnssDatatStatus.active;
	messageOut->GnssDatatStatus.portNum = messageIn.GnssDatatStatus.portNum;
	messageOut->GnssDatatStatus.portType = static_cast<port_type_t>(messageIn.GnssDatatStatus.portType.value);
	messageOut->GnssDatatStatus.active = messageIn.GnssDatatStatus.active;

	messageOut->PositionStatus.x_axis = messageIn.PositionStatus.x_axis;
	messageOut->PositionStatus.y_axis = messageIn.PositionStatus.y_axis;
	messageOut->PositionStatus.z_axis = messageIn.PositionStatus.z_axis;
	messageOut->PositionStatus.enabled = messageIn.PositionStatus.enabled;
	messageOut->PositionStatus.x_axis = messageIn.PositionStatus.x_axis;
	messageOut->PositionStatus.y_axis = messageIn.PositionStatus.y_axis;
	messageOut->PositionStatus.z_axis = messageIn.PositionStatus.z_axis;
	messageOut->PositionStatus.enabled = messageIn.PositionStatus.enabled;

	messageOut->VelocityStatus.x_axis = messageIn.VelocityStatus.x_axis;
	messageOut->VelocityStatus.y_axis = messageIn.VelocityStatus.y_axis;
	messageOut->VelocityStatus.z_axis = messageIn.VelocityStatus.z_axis;
	messageOut->VelocityStatus.enabled = messageIn.VelocityStatus.enabled;
	messageOut->VelocityStatus.x_axis = messageIn.VelocityStatus.x_axis;
	messageOut->VelocityStatus.y_axis = messageIn.VelocityStatus.y_axis;
	messageOut->VelocityStatus.z_axis = messageIn.VelocityStatus.z_axis;
	messageOut->VelocityStatus.enabled = messageIn.VelocityStatus.enabled;

	messageOut->AttitudeStatus.x_axis = messageIn.AttitudeStatus.x_axis;
	messageOut->AttitudeStatus.y_axis = messageIn.AttitudeStatus.y_axis;
	messageOut->AttitudeStatus.z_axis = messageIn.AttitudeStatus.z_axis;
	messageOut->AttitudeStatus.enabled = messageIn.AttitudeStatus.enabled;
	messageOut->AttitudeStatus.x_axis = messageIn.AttitudeStatus.x_axis;
	messageOut->AttitudeStatus.y_axis = messageIn.AttitudeStatus.y_axis;
	messageOut->AttitudeStatus.z_axis = messageIn.AttitudeStatus.z_axis;
	messageOut->AttitudeStatus.enabled = messageIn.AttitudeStatus.enabled;
	messageOut->NavAxesSelected = messageIn.NavAxesSelected;
}

void Msg_6651_pub_callback(uint8_t * buffer)
{
	Msg_6651 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6651 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6651);
	ROS_DEBUG("Message 0x6651 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6651_pub_initialized == false){
		init_6651(getRosHandle());}
	// Publish the message
	Msg_6651_pub.publish(msgStruct_6651);
	return;
}
