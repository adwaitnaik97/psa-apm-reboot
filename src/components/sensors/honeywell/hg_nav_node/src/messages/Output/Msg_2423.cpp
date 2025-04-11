#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2423.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2423 msgStruct_2423;

bool Msg_2423_pub_initialized = false;

ros::Publisher Msg_2423_pub;
void init_2423(ros::NodeHandle * n){
	Msg_2423_pub = n->advertise<hg_nav_node::Msg_2423>(MSG_2423_PATH, 5);
	Msg_2423_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2423_PATH);
	return;
}

void stop_2423(void){
	Msg_2423_pub.shutdown();
	Msg_2423_pub_initialized = false;
	ROS_INFO("0x2423 stopped");
	return;
}

// Msg_2423 to Topic
void convert(Msg_2423 messageIn, hg_nav_node::Msg_2423 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gyro_bias_x_std_dev = messageIn.gyro_bias_x_std_dev;
	messageOut->gyro_bias_y_std_dev = messageIn.gyro_bias_y_std_dev;
	messageOut->gyro_bias_z_std_dev = messageIn.gyro_bias_z_std_dev;
	messageOut->gyro_bias_inrun_x_std_dev = messageIn.gyro_bias_inrun_x_std_dev;
	messageOut->gyro_bias_inrun_y_std_dev = messageIn.gyro_bias_inrun_y_std_dev;
	messageOut->gyro_bias_inrun_z_std_dev = messageIn.gyro_bias_inrun_z_std_dev;
	messageOut->gyro_scale_factor_x_std_dev = messageIn.gyro_scale_factor_x_std_dev;
	messageOut->gyro_scale_factor_y_std_dev = messageIn.gyro_scale_factor_y_std_dev;
	messageOut->gyro_scale_factor_z_std_dev = messageIn.gyro_scale_factor_z_std_dev;
	messageOut->gyro_nonorthogonality_yz_std_dev = messageIn.gyro_nonorthogonality_yz_std_dev;
	messageOut->gyro_nonorthogonality_zx_std_dev = messageIn.gyro_nonorthogonality_zx_std_dev;
	messageOut->gyro_nonorthogonality_xy_std_dev = messageIn.gyro_nonorthogonality_xy_std_dev;
	messageOut->accelerometer_bias_x_std_dev = messageIn.accelerometer_bias_x_std_dev;
	messageOut->accelerometer_bias_y_std_dev = messageIn.accelerometer_bias_y_std_dev;
	messageOut->accelerometer_bias_z_std_dev = messageIn.accelerometer_bias_z_std_dev;
	messageOut->accelerometer_bias_inrun_x_std_dev = messageIn.accelerometer_bias_inrun_x_std_dev;
	messageOut->accelerometer_bias_inrun_y_std_dev = messageIn.accelerometer_bias_inrun_y_std_dev;
	messageOut->accelerometer_bias_inrun_z_std_dev = messageIn.accelerometer_bias_inrun_z_std_dev;
	messageOut->accelerometer_scale_factor_x_std_dev = messageIn.accelerometer_scale_factor_x_std_dev;
	messageOut->accelerometer_scale_factor_y_std_dev = messageIn.accelerometer_scale_factor_y_std_dev;
	messageOut->accelerometer_scale_factor_z_std_dev = messageIn.accelerometer_scale_factor_z_std_dev;
	messageOut->accelerometer_nonorthogonality_yz_std_dev = messageIn.accelerometer_nonorthogonality_yz_std_dev;
	messageOut->accelerometer_nonorthogonality_zx_std_dev = messageIn.accelerometer_nonorthogonality_zx_std_dev;
	messageOut->accelerometer_nonorthogonality_xy_std_dev = messageIn.accelerometer_nonorthogonality_xy_std_dev;
	messageOut->accelerometer_misalignment_x_std_dev = messageIn.accelerometer_misalignment_x_std_dev;
	messageOut->accelerometer_misalignment_y_std_dev = messageIn.accelerometer_misalignment_y_std_dev;
	messageOut->accelerometer_misalignment_z_std_dev = messageIn.accelerometer_misalignment_z_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_x_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_x_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_y_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_y_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_z_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_z_std_dev;
}

// Topic to Msg_2423
void convert(hg_nav_node::Msg_2423 messageIn, Msg_2423 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gyro_bias_x_std_dev = messageIn.gyro_bias_x_std_dev;
	messageOut->gyro_bias_y_std_dev = messageIn.gyro_bias_y_std_dev;
	messageOut->gyro_bias_z_std_dev = messageIn.gyro_bias_z_std_dev;
	messageOut->gyro_bias_inrun_x_std_dev = messageIn.gyro_bias_inrun_x_std_dev;
	messageOut->gyro_bias_inrun_y_std_dev = messageIn.gyro_bias_inrun_y_std_dev;
	messageOut->gyro_bias_inrun_z_std_dev = messageIn.gyro_bias_inrun_z_std_dev;
	messageOut->gyro_scale_factor_x_std_dev = messageIn.gyro_scale_factor_x_std_dev;
	messageOut->gyro_scale_factor_y_std_dev = messageIn.gyro_scale_factor_y_std_dev;
	messageOut->gyro_scale_factor_z_std_dev = messageIn.gyro_scale_factor_z_std_dev;
	messageOut->gyro_nonorthogonality_yz_std_dev = messageIn.gyro_nonorthogonality_yz_std_dev;
	messageOut->gyro_nonorthogonality_zx_std_dev = messageIn.gyro_nonorthogonality_zx_std_dev;
	messageOut->gyro_nonorthogonality_xy_std_dev = messageIn.gyro_nonorthogonality_xy_std_dev;
	messageOut->accelerometer_bias_x_std_dev = messageIn.accelerometer_bias_x_std_dev;
	messageOut->accelerometer_bias_y_std_dev = messageIn.accelerometer_bias_y_std_dev;
	messageOut->accelerometer_bias_z_std_dev = messageIn.accelerometer_bias_z_std_dev;
	messageOut->accelerometer_bias_inrun_x_std_dev = messageIn.accelerometer_bias_inrun_x_std_dev;
	messageOut->accelerometer_bias_inrun_y_std_dev = messageIn.accelerometer_bias_inrun_y_std_dev;
	messageOut->accelerometer_bias_inrun_z_std_dev = messageIn.accelerometer_bias_inrun_z_std_dev;
	messageOut->accelerometer_scale_factor_x_std_dev = messageIn.accelerometer_scale_factor_x_std_dev;
	messageOut->accelerometer_scale_factor_y_std_dev = messageIn.accelerometer_scale_factor_y_std_dev;
	messageOut->accelerometer_scale_factor_z_std_dev = messageIn.accelerometer_scale_factor_z_std_dev;
	messageOut->accelerometer_nonorthogonality_yz_std_dev = messageIn.accelerometer_nonorthogonality_yz_std_dev;
	messageOut->accelerometer_nonorthogonality_zx_std_dev = messageIn.accelerometer_nonorthogonality_zx_std_dev;
	messageOut->accelerometer_nonorthogonality_xy_std_dev = messageIn.accelerometer_nonorthogonality_xy_std_dev;
	messageOut->accelerometer_misalignment_x_std_dev = messageIn.accelerometer_misalignment_x_std_dev;
	messageOut->accelerometer_misalignment_y_std_dev = messageIn.accelerometer_misalignment_y_std_dev;
	messageOut->accelerometer_misalignment_z_std_dev = messageIn.accelerometer_misalignment_z_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_x_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_x_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_y_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_y_std_dev;
	messageOut->accelerometer_scale_factor_nonlinearity_z_std_dev = messageIn.accelerometer_scale_factor_nonlinearity_z_std_dev;
}

void Msg_2423_pub_callback(uint8_t * buffer)
{
	Msg_2423 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2423 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2423);
	ROS_DEBUG("Message 0x2423 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2423_pub_initialized == false){
		init_2423(getRosHandle());}
	// Publish the message
	Msg_2423_pub.publish(msgStruct_2423);
	return;
}
