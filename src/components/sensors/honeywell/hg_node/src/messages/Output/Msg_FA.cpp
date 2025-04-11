#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_FA.h>
#include <hg_node/sensor_axes_t.h>
hg_node::Msg_FA msgStruct_FA;

bool Msg_FA_pub_initialized = false;

ros::Publisher Msg_FA_pub;
void init_FA(ros::NodeHandle * n){
	Msg_FA_pub = n->advertise<hg_node::Msg_FA>(MSG_FA_PATH, 5);
	Msg_FA_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_FA_PATH);
	return;
}

void stop_FA(void){
	if (Msg_FA_pub_initialized){
		Msg_FA_pub.shutdown();
		Msg_FA_pub_initialized = false;
		ROS_INFO("0xFA stopped");
	}
	return;
}

// Msg_FA to Topic
void convert(Msg_FA messageIn, hg_node::Msg_FA * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;

	messageOut->active_sensor_axes.accel_x = messageIn.active_sensor_axes.accel_x;
	messageOut->active_sensor_axes.accel_y = messageIn.active_sensor_axes.accel_y;
	messageOut->active_sensor_axes.accel_z = messageIn.active_sensor_axes.accel_z;
	messageOut->active_sensor_axes.gyro_x = messageIn.active_sensor_axes.gyro_x;
	messageOut->active_sensor_axes.gyro_y = messageIn.active_sensor_axes.gyro_y;
	messageOut->active_sensor_axes.gyro_z = messageIn.active_sensor_axes.gyro_z;
	messageOut->active_sensor_axes.mag_x = messageIn.active_sensor_axes.mag_x;
	messageOut->active_sensor_axes.mag_y = messageIn.active_sensor_axes.mag_y;
	messageOut->active_sensor_axes.mag_z = messageIn.active_sensor_axes.mag_z;

	messageOut->saturated_sensor_axes.accel_x = messageIn.saturated_sensor_axes.accel_x;
	messageOut->saturated_sensor_axes.accel_y = messageIn.saturated_sensor_axes.accel_y;
	messageOut->saturated_sensor_axes.accel_z = messageIn.saturated_sensor_axes.accel_z;
	messageOut->saturated_sensor_axes.gyro_x = messageIn.saturated_sensor_axes.gyro_x;
	messageOut->saturated_sensor_axes.gyro_y = messageIn.saturated_sensor_axes.gyro_y;
	messageOut->saturated_sensor_axes.gyro_z = messageIn.saturated_sensor_axes.gyro_z;
	messageOut->saturated_sensor_axes.mag_x = messageIn.saturated_sensor_axes.mag_x;
	messageOut->saturated_sensor_axes.mag_y = messageIn.saturated_sensor_axes.mag_y;
	messageOut->saturated_sensor_axes.mag_z = messageIn.saturated_sensor_axes.mag_z;

	messageOut->sensor_stat_fail.accel_x = messageIn.sensor_stat_fail.accel_x;
	messageOut->sensor_stat_fail.accel_y = messageIn.sensor_stat_fail.accel_y;
	messageOut->sensor_stat_fail.accel_z = messageIn.sensor_stat_fail.accel_z;
	messageOut->sensor_stat_fail.gyro_x = messageIn.sensor_stat_fail.gyro_x;
	messageOut->sensor_stat_fail.gyro_y = messageIn.sensor_stat_fail.gyro_y;
	messageOut->sensor_stat_fail.gyro_z = messageIn.sensor_stat_fail.gyro_z;
	messageOut->sensor_stat_fail.mag_x = messageIn.sensor_stat_fail.mag_x;
	messageOut->sensor_stat_fail.mag_y = messageIn.sensor_stat_fail.mag_y;
	messageOut->sensor_stat_fail.mag_z = messageIn.sensor_stat_fail.mag_z;

	messageOut->sensor_temp_fail.accel_x = messageIn.sensor_temp_fail.accel_x;
	messageOut->sensor_temp_fail.accel_y = messageIn.sensor_temp_fail.accel_y;
	messageOut->sensor_temp_fail.accel_z = messageIn.sensor_temp_fail.accel_z;
	messageOut->sensor_temp_fail.gyro_x = messageIn.sensor_temp_fail.gyro_x;
	messageOut->sensor_temp_fail.gyro_y = messageIn.sensor_temp_fail.gyro_y;
	messageOut->sensor_temp_fail.gyro_z = messageIn.sensor_temp_fail.gyro_z;
	messageOut->sensor_temp_fail.mag_x = messageIn.sensor_temp_fail.mag_x;
	messageOut->sensor_temp_fail.mag_y = messageIn.sensor_temp_fail.mag_y;
	messageOut->sensor_temp_fail.mag_z = messageIn.sensor_temp_fail.mag_z;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_FA
void convert(hg_node::Msg_FA messageIn, Msg_FA * messageOut)
{

	messageOut->active_sensor_axes.accel_x = messageIn.active_sensor_axes.accel_x;
	messageOut->active_sensor_axes.accel_y = messageIn.active_sensor_axes.accel_y;
	messageOut->active_sensor_axes.accel_z = messageIn.active_sensor_axes.accel_z;
	messageOut->active_sensor_axes.gyro_x = messageIn.active_sensor_axes.gyro_x;
	messageOut->active_sensor_axes.gyro_y = messageIn.active_sensor_axes.gyro_y;
	messageOut->active_sensor_axes.gyro_z = messageIn.active_sensor_axes.gyro_z;
	messageOut->active_sensor_axes.mag_x = messageIn.active_sensor_axes.mag_x;
	messageOut->active_sensor_axes.mag_y = messageIn.active_sensor_axes.mag_y;
	messageOut->active_sensor_axes.mag_z = messageIn.active_sensor_axes.mag_z;

	messageOut->saturated_sensor_axes.accel_x = messageIn.saturated_sensor_axes.accel_x;
	messageOut->saturated_sensor_axes.accel_y = messageIn.saturated_sensor_axes.accel_y;
	messageOut->saturated_sensor_axes.accel_z = messageIn.saturated_sensor_axes.accel_z;
	messageOut->saturated_sensor_axes.gyro_x = messageIn.saturated_sensor_axes.gyro_x;
	messageOut->saturated_sensor_axes.gyro_y = messageIn.saturated_sensor_axes.gyro_y;
	messageOut->saturated_sensor_axes.gyro_z = messageIn.saturated_sensor_axes.gyro_z;
	messageOut->saturated_sensor_axes.mag_x = messageIn.saturated_sensor_axes.mag_x;
	messageOut->saturated_sensor_axes.mag_y = messageIn.saturated_sensor_axes.mag_y;
	messageOut->saturated_sensor_axes.mag_z = messageIn.saturated_sensor_axes.mag_z;

	messageOut->sensor_stat_fail.accel_x = messageIn.sensor_stat_fail.accel_x;
	messageOut->sensor_stat_fail.accel_y = messageIn.sensor_stat_fail.accel_y;
	messageOut->sensor_stat_fail.accel_z = messageIn.sensor_stat_fail.accel_z;
	messageOut->sensor_stat_fail.gyro_x = messageIn.sensor_stat_fail.gyro_x;
	messageOut->sensor_stat_fail.gyro_y = messageIn.sensor_stat_fail.gyro_y;
	messageOut->sensor_stat_fail.gyro_z = messageIn.sensor_stat_fail.gyro_z;
	messageOut->sensor_stat_fail.mag_x = messageIn.sensor_stat_fail.mag_x;
	messageOut->sensor_stat_fail.mag_y = messageIn.sensor_stat_fail.mag_y;
	messageOut->sensor_stat_fail.mag_z = messageIn.sensor_stat_fail.mag_z;

	messageOut->sensor_temp_fail.accel_x = messageIn.sensor_temp_fail.accel_x;
	messageOut->sensor_temp_fail.accel_y = messageIn.sensor_temp_fail.accel_y;
	messageOut->sensor_temp_fail.accel_z = messageIn.sensor_temp_fail.accel_z;
	messageOut->sensor_temp_fail.gyro_x = messageIn.sensor_temp_fail.gyro_x;
	messageOut->sensor_temp_fail.gyro_y = messageIn.sensor_temp_fail.gyro_y;
	messageOut->sensor_temp_fail.gyro_z = messageIn.sensor_temp_fail.gyro_z;
	messageOut->sensor_temp_fail.mag_x = messageIn.sensor_temp_fail.mag_x;
	messageOut->sensor_temp_fail.mag_y = messageIn.sensor_temp_fail.mag_y;
	messageOut->sensor_temp_fail.mag_z = messageIn.sensor_temp_fail.mag_z;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_FA_pub_callback(uint8_t * buffer)
{
	Msg_FA Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xFA deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_FA);
	ROS_DEBUG("Message 0xFA Received");

	// Initialize Publisher if not initialized yet
	if (Msg_FA_pub_initialized == false){
		init_FA(getRosHandle());}
	// Publish the message
	Msg_FA_pub.publish(msgStruct_FA);
	return;
}
