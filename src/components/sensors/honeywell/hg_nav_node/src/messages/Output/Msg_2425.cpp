#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2425.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2425 msgStruct_2425;

bool Msg_2425_pub_initialized = false;

ros::Publisher Msg_2425_pub;
void init_2425(ros::NodeHandle * n){
	Msg_2425_pub = n->advertise<hg_nav_node::Msg_2425>(MSG_2425_PATH, 5);
	Msg_2425_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2425_PATH);
	return;
}

void stop_2425(void){
	Msg_2425_pub.shutdown();
	Msg_2425_pub_initialized = false;
	ROS_INFO("0x2425 stopped");
	return;
}

// Msg_2425 to Topic
void convert(Msg_2425 messageIn, hg_nav_node::Msg_2425 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->TA_PosAidingX_ErrEst = messageIn.TA_PosAidingX_ErrEst;
	messageOut->TA_PosAidingY_ErrEst = messageIn.TA_PosAidingY_ErrEst;
	messageOut->TA_PosAidingZ_ErrEst = messageIn.TA_PosAidingZ_ErrEst;
	messageOut->TA_VelAidingX_ErrEst = messageIn.TA_VelAidingX_ErrEst;
	messageOut->TA_VelAidingY_ErrEst = messageIn.TA_VelAidingY_ErrEst;
	messageOut->TA_VelAidingZ_ErrEst = messageIn.TA_VelAidingZ_ErrEst;
	messageOut->TA_LeverArmX_ErrEst = messageIn.TA_LeverArmX_ErrEst;
	messageOut->TA_LeverArmY_ErrEst = messageIn.TA_LeverArmY_ErrEst;
	messageOut->TA_LeverArmZ_ErrEst = messageIn.TA_LeverArmZ_ErrEst;
	messageOut->TA_BoresightX_ErrEst = messageIn.TA_BoresightX_ErrEst;
	messageOut->TA_BoresightY_ErrEst = messageIn.TA_BoresightY_ErrEst;
	messageOut->TA_BoresightZ_ErrEst = messageIn.TA_BoresightZ_ErrEst;
	messageOut->TA_PosAidingX_Stdv = messageIn.TA_PosAidingX_Stdv;
	messageOut->TA_PosAidingY_Stdv = messageIn.TA_PosAidingY_Stdv;
	messageOut->TA_PosAidingZ_Stdv = messageIn.TA_PosAidingZ_Stdv;
	messageOut->TA_VelAidingX_Stdv = messageIn.TA_VelAidingX_Stdv;
	messageOut->TA_VelAidingY_Stdv = messageIn.TA_VelAidingY_Stdv;
	messageOut->TA_VelAidingZ_Stdv = messageIn.TA_VelAidingZ_Stdv;
	messageOut->TA_LeverArmX_Stdv = messageIn.TA_LeverArmX_Stdv;
	messageOut->TA_LeverArmY_Stdv = messageIn.TA_LeverArmY_Stdv;
	messageOut->TA_LeverArmZ_Stdv = messageIn.TA_LeverArmZ_Stdv;
	messageOut->TA_BoresightX_Stdv = messageIn.TA_BoresightX_Stdv;
	messageOut->TA_BoresightY_Stdv = messageIn.TA_BoresightY_Stdv;
	messageOut->TA_BoresightZ_Stdv = messageIn.TA_BoresightZ_Stdv;
}

// Topic to Msg_2425
void convert(hg_nav_node::Msg_2425 messageIn, Msg_2425 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->TA_PosAidingX_ErrEst = messageIn.TA_PosAidingX_ErrEst;
	messageOut->TA_PosAidingY_ErrEst = messageIn.TA_PosAidingY_ErrEst;
	messageOut->TA_PosAidingZ_ErrEst = messageIn.TA_PosAidingZ_ErrEst;
	messageOut->TA_VelAidingX_ErrEst = messageIn.TA_VelAidingX_ErrEst;
	messageOut->TA_VelAidingY_ErrEst = messageIn.TA_VelAidingY_ErrEst;
	messageOut->TA_VelAidingZ_ErrEst = messageIn.TA_VelAidingZ_ErrEst;
	messageOut->TA_LeverArmX_ErrEst = messageIn.TA_LeverArmX_ErrEst;
	messageOut->TA_LeverArmY_ErrEst = messageIn.TA_LeverArmY_ErrEst;
	messageOut->TA_LeverArmZ_ErrEst = messageIn.TA_LeverArmZ_ErrEst;
	messageOut->TA_BoresightX_ErrEst = messageIn.TA_BoresightX_ErrEst;
	messageOut->TA_BoresightY_ErrEst = messageIn.TA_BoresightY_ErrEst;
	messageOut->TA_BoresightZ_ErrEst = messageIn.TA_BoresightZ_ErrEst;
	messageOut->TA_PosAidingX_Stdv = messageIn.TA_PosAidingX_Stdv;
	messageOut->TA_PosAidingY_Stdv = messageIn.TA_PosAidingY_Stdv;
	messageOut->TA_PosAidingZ_Stdv = messageIn.TA_PosAidingZ_Stdv;
	messageOut->TA_VelAidingX_Stdv = messageIn.TA_VelAidingX_Stdv;
	messageOut->TA_VelAidingY_Stdv = messageIn.TA_VelAidingY_Stdv;
	messageOut->TA_VelAidingZ_Stdv = messageIn.TA_VelAidingZ_Stdv;
	messageOut->TA_LeverArmX_Stdv = messageIn.TA_LeverArmX_Stdv;
	messageOut->TA_LeverArmY_Stdv = messageIn.TA_LeverArmY_Stdv;
	messageOut->TA_LeverArmZ_Stdv = messageIn.TA_LeverArmZ_Stdv;
	messageOut->TA_BoresightX_Stdv = messageIn.TA_BoresightX_Stdv;
	messageOut->TA_BoresightY_Stdv = messageIn.TA_BoresightY_Stdv;
	messageOut->TA_BoresightZ_Stdv = messageIn.TA_BoresightZ_Stdv;
}

void Msg_2425_pub_callback(uint8_t * buffer)
{
	Msg_2425 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2425 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2425);
	ROS_DEBUG("Message 0x2425 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2425_pub_initialized == false){
		init_2425(getRosHandle());}
	// Publish the message
	Msg_2425_pub.publish(msgStruct_2425);
	return;
}
