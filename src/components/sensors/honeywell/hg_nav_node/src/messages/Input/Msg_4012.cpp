#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_4012.h>
#include <hg_nav_node/restart_nav_config_t.h>
hg_nav_node::Msg_4012 msgStruct_4012;

ros::Subscriber Msg_4012_sub;
void init_4012(ros::NodeHandle * n){
	Msg_4012_sub = n->subscribe(MSG_4012_PATH, 5, Msg_4012_sub_callback);
	ROS_INFO("Starting sub %s",MSG_4012_PATH);
	return;
}

void stop_4012(void){
	Msg_4012_sub.shutdown();
	ROS_INFO("0x4012 stopped");
	return;
}

// Msg_4012 to Topic
void convert(Msg_4012 messageIn, hg_nav_node::Msg_4012 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->nav_init_config.gyro_bias = messageIn.nav_init_config.gyro_bias;
	messageOut->nav_init_config.accel_bias = messageIn.nav_init_config.accel_bias;
	messageOut->nav_init_config.gyro_sf = messageIn.nav_init_config.gyro_sf;
	messageOut->nav_init_config.accel_sf = messageIn.nav_init_config.accel_sf;
	messageOut->nav_init_config.main_ant_lev = messageIn.nav_init_config.main_ant_lev;
	messageOut->nav_init_config.aux_ant_lev = messageIn.nav_init_config.aux_ant_lev;
	messageOut->nav_init_config.nav_init = messageIn.nav_init_config.nav_init;
	messageOut->nav_init_config.gyro_bias = messageIn.nav_init_config.gyro_bias;
	messageOut->nav_init_config.accel_bias = messageIn.nav_init_config.accel_bias;
	messageOut->nav_init_config.gyro_sf = messageIn.nav_init_config.gyro_sf;
	messageOut->nav_init_config.accel_sf = messageIn.nav_init_config.accel_sf;
	messageOut->nav_init_config.main_ant_lev = messageIn.nav_init_config.main_ant_lev;
	messageOut->nav_init_config.aux_ant_lev = messageIn.nav_init_config.aux_ant_lev;
	messageOut->nav_init_config.nav_init = messageIn.nav_init_config.nav_init;
	messageOut->Sigma_Latitude = messageIn.Sigma_Latitude;
	messageOut->Sigma_Longitude = messageIn.Sigma_Longitude;
	messageOut->Sigma_Altitude = messageIn.Sigma_Altitude;
	messageOut->Sigma_Heading = messageIn.Sigma_Heading;
}

// Topic to Msg_4012
void convert(hg_nav_node::Msg_4012 messageIn, Msg_4012 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->nav_init_config.gyro_bias = messageIn.nav_init_config.gyro_bias;
	messageOut->nav_init_config.accel_bias = messageIn.nav_init_config.accel_bias;
	messageOut->nav_init_config.gyro_sf = messageIn.nav_init_config.gyro_sf;
	messageOut->nav_init_config.accel_sf = messageIn.nav_init_config.accel_sf;
	messageOut->nav_init_config.main_ant_lev = messageIn.nav_init_config.main_ant_lev;
	messageOut->nav_init_config.aux_ant_lev = messageIn.nav_init_config.aux_ant_lev;
	messageOut->nav_init_config.nav_init = messageIn.nav_init_config.nav_init;
	messageOut->nav_init_config.gyro_bias = messageIn.nav_init_config.gyro_bias;
	messageOut->nav_init_config.accel_bias = messageIn.nav_init_config.accel_bias;
	messageOut->nav_init_config.gyro_sf = messageIn.nav_init_config.gyro_sf;
	messageOut->nav_init_config.accel_sf = messageIn.nav_init_config.accel_sf;
	messageOut->nav_init_config.main_ant_lev = messageIn.nav_init_config.main_ant_lev;
	messageOut->nav_init_config.aux_ant_lev = messageIn.nav_init_config.aux_ant_lev;
	messageOut->nav_init_config.nav_init = messageIn.nav_init_config.nav_init;
	messageOut->Sigma_Latitude = messageIn.Sigma_Latitude;
	messageOut->Sigma_Longitude = messageIn.Sigma_Longitude;
	messageOut->Sigma_Altitude = messageIn.Sigma_Altitude;
	messageOut->Sigma_Heading = messageIn.Sigma_Heading;
}

void Msg_4012_sub_callback(const hg_nav_node::Msg_4012::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_4012 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x4012 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x4012 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
