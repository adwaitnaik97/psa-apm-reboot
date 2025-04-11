#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6438.h>
hg_nav_node::Msg_6438 msgStruct_6438;

bool Msg_6438_pub_initialized = false;

ros::Publisher Msg_6438_pub;
void init_6438(ros::NodeHandle * n){
	Msg_6438_pub = n->advertise<hg_nav_node::Msg_6438>(MSG_6438_PATH, 5);
	Msg_6438_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6438_PATH);
	return;
}

void stop_6438(void){
	Msg_6438_pub.shutdown();
	Msg_6438_pub_initialized = false;
	ROS_INFO("0x6438 stopped");
	return;
}

// Msg_6438 to Topic
void convert(Msg_6438 messageIn, hg_nav_node::Msg_6438 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->residual_x = messageIn.residual_x;
	messageOut->residual_y = messageIn.residual_y;
	messageOut->residual_z = messageIn.residual_z;
	messageOut->scalefactor_correction = messageIn.scalefactor_correction;
	messageOut->scalefactor_correction_pitchsens = messageIn.scalefactor_correction_pitchsens;
	messageOut->scalefactor_correction_pitchratesens = messageIn.scalefactor_correction_pitchratesens;
	messageOut->boresight_yaw = messageIn.boresight_yaw;
	messageOut->boresight_yaw_rollsens = messageIn.boresight_yaw_rollsens;
	messageOut->boresight_yaw_rocksens = messageIn.boresight_yaw_rocksens;
	messageOut->boresight_pitch = messageIn.boresight_pitch;
	messageOut->boresight_pitch_rollsens = messageIn.boresight_pitch_rollsens;
	messageOut->boresight_pitch_rocksens = messageIn.boresight_pitch_rocksens;
	messageOut->lever_arm_x = messageIn.lever_arm_x;
	messageOut->lever_arm_y = messageIn.lever_arm_y;
	messageOut->lever_arm_z = messageIn.lever_arm_z;
	messageOut->scalefactor_correction_stdv = messageIn.scalefactor_correction_stdv;
	messageOut->scalefactor_correction_pitchsens_stdv = messageIn.scalefactor_correction_pitchsens_stdv;
	messageOut->scalefactor_correction_pitchratesens_stdv = messageIn.scalefactor_correction_pitchratesens_stdv;
	messageOut->boresight_yaw_stdv = messageIn.boresight_yaw_stdv;
	messageOut->boresight_yaw_rollsens_stdv = messageIn.boresight_yaw_rollsens_stdv;
	messageOut->boresight_yaw_rocksens_stdv = messageIn.boresight_yaw_rocksens_stdv;
	messageOut->boresight_pitch_stdv = messageIn.boresight_pitch_stdv;
	messageOut->boresight_pitch_rollsens_stdv = messageIn.boresight_pitch_rollsens_stdv;
	messageOut->boresight_pitch_rocksens_stdv = messageIn.boresight_pitch_rocksens_stdv;
	messageOut->lever_arm_stdv_x = messageIn.lever_arm_stdv_x;
	messageOut->lever_arm_stdv_y = messageIn.lever_arm_stdv_y;
	messageOut->lever_arm_stdv_z = messageIn.lever_arm_stdv_z;
	messageOut->stored_lever_arm_x = messageIn.stored_lever_arm_x;
	messageOut->stored_lever_arm_y = messageIn.stored_lever_arm_y;
	messageOut->stored_lever_arm_z = messageIn.stored_lever_arm_z;
	messageOut->boresight_roll = messageIn.boresight_roll;
	messageOut->boresight_stdv_roll = messageIn.boresight_stdv_roll;
}

// Topic to Msg_6438
void convert(hg_nav_node::Msg_6438 messageIn, Msg_6438 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->residual_x = messageIn.residual_x;
	messageOut->residual_y = messageIn.residual_y;
	messageOut->residual_z = messageIn.residual_z;
	messageOut->scalefactor_correction = messageIn.scalefactor_correction;
	messageOut->scalefactor_correction_pitchsens = messageIn.scalefactor_correction_pitchsens;
	messageOut->scalefactor_correction_pitchratesens = messageIn.scalefactor_correction_pitchratesens;
	messageOut->boresight_yaw = messageIn.boresight_yaw;
	messageOut->boresight_yaw_rollsens = messageIn.boresight_yaw_rollsens;
	messageOut->boresight_yaw_rocksens = messageIn.boresight_yaw_rocksens;
	messageOut->boresight_pitch = messageIn.boresight_pitch;
	messageOut->boresight_pitch_rollsens = messageIn.boresight_pitch_rollsens;
	messageOut->boresight_pitch_rocksens = messageIn.boresight_pitch_rocksens;
	messageOut->lever_arm_x = messageIn.lever_arm_x;
	messageOut->lever_arm_y = messageIn.lever_arm_y;
	messageOut->lever_arm_z = messageIn.lever_arm_z;
	messageOut->scalefactor_correction_stdv = messageIn.scalefactor_correction_stdv;
	messageOut->scalefactor_correction_pitchsens_stdv = messageIn.scalefactor_correction_pitchsens_stdv;
	messageOut->scalefactor_correction_pitchratesens_stdv = messageIn.scalefactor_correction_pitchratesens_stdv;
	messageOut->boresight_yaw_stdv = messageIn.boresight_yaw_stdv;
	messageOut->boresight_yaw_rollsens_stdv = messageIn.boresight_yaw_rollsens_stdv;
	messageOut->boresight_yaw_rocksens_stdv = messageIn.boresight_yaw_rocksens_stdv;
	messageOut->boresight_pitch_stdv = messageIn.boresight_pitch_stdv;
	messageOut->boresight_pitch_rollsens_stdv = messageIn.boresight_pitch_rollsens_stdv;
	messageOut->boresight_pitch_rocksens_stdv = messageIn.boresight_pitch_rocksens_stdv;
	messageOut->lever_arm_stdv_x = messageIn.lever_arm_stdv_x;
	messageOut->lever_arm_stdv_y = messageIn.lever_arm_stdv_y;
	messageOut->lever_arm_stdv_z = messageIn.lever_arm_stdv_z;
	messageOut->stored_lever_arm_x = messageIn.stored_lever_arm_x;
	messageOut->stored_lever_arm_y = messageIn.stored_lever_arm_y;
	messageOut->stored_lever_arm_z = messageIn.stored_lever_arm_z;
	messageOut->boresight_roll = messageIn.boresight_roll;
	messageOut->boresight_stdv_roll = messageIn.boresight_stdv_roll;
}

void Msg_6438_pub_callback(uint8_t * buffer)
{
	Msg_6438 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6438 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6438);
	ROS_DEBUG("Message 0x6438 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6438_pub_initialized == false){
		init_6438(getRosHandle());}
	// Publish the message
	Msg_6438_pub.publish(msgStruct_6438);
	return;
}
