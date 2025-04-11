#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2411.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2411 msgStruct_2411;

bool Msg_2411_pub_initialized = false;

ros::Publisher Msg_2411_pub;
void init_2411(ros::NodeHandle * n){
	Msg_2411_pub = n->advertise<hg_nav_node::Msg_2411>(MSG_2411_PATH, 5);
	Msg_2411_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2411_PATH);
	return;
}

void stop_2411(void){
	Msg_2411_pub.shutdown();
	Msg_2411_pub_initialized = false;
	ROS_INFO("0x2411 stopped");
	return;
}

// Msg_2411 to Topic
void convert(Msg_2411 messageIn, hg_nav_node::Msg_2411 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gyro_bias_error_x = messageIn.gyro_bias_error_x;
	messageOut->gyro_bias_error_y = messageIn.gyro_bias_error_y;
	messageOut->gyro_bias_error_z = messageIn.gyro_bias_error_z;
	messageOut->gyro_bias_inrun_error_x = messageIn.gyro_bias_inrun_error_x;
	messageOut->gyro_bias_inrun_error_y = messageIn.gyro_bias_inrun_error_y;
	messageOut->gyro_bias_inrun_error_z = messageIn.gyro_bias_inrun_error_z;
	messageOut->gyro_scale_factor_error_x = messageIn.gyro_scale_factor_error_x;
	messageOut->gyro_scale_factor_error_y = messageIn.gyro_scale_factor_error_y;
	messageOut->gyro_scale_factor_error_z = messageIn.gyro_scale_factor_error_z;
	messageOut->gyro_nonorthogonality_error_yz = messageIn.gyro_nonorthogonality_error_yz;
	messageOut->gyro_nonorthogonality_error_zx = messageIn.gyro_nonorthogonality_error_zx;
	messageOut->gyro_nonorthogonality_error_xy = messageIn.gyro_nonorthogonality_error_xy;
	messageOut->accelerometer_bias_error_x = messageIn.accelerometer_bias_error_x;
	messageOut->accelerometer_bias_error_y = messageIn.accelerometer_bias_error_y;
	messageOut->accelerometer_bias_error_z = messageIn.accelerometer_bias_error_z;
	messageOut->accelerometer_bias_inrun_error_x = messageIn.accelerometer_bias_inrun_error_x;
	messageOut->accelerometer_bias_inrun_error_y = messageIn.accelerometer_bias_inrun_error_y;
	messageOut->accelerometer_bias_inrun_error_z = messageIn.accelerometer_bias_inrun_error_z;
	messageOut->accelerometer_scale_factor_error_x = messageIn.accelerometer_scale_factor_error_x;
	messageOut->accelerometer_scale_factor_error_y = messageIn.accelerometer_scale_factor_error_y;
	messageOut->accelerometer_scale_factor_error_z = messageIn.accelerometer_scale_factor_error_z;
	messageOut->accelerometer_nonorthogonality_error_yz = messageIn.accelerometer_nonorthogonality_error_yz;
	messageOut->accelerometer_nonorthogonality_error_zx = messageIn.accelerometer_nonorthogonality_error_zx;
	messageOut->accelerometer_nonorthogonality_error_xy = messageIn.accelerometer_nonorthogonality_error_xy;
	messageOut->accelerometer_misalignment_error_x = messageIn.accelerometer_misalignment_error_x;
	messageOut->accelerometer_misalignment_error_y = messageIn.accelerometer_misalignment_error_y;
	messageOut->accelerometer_misalignment_error_z = messageIn.accelerometer_misalignment_error_z;
	messageOut->accelerometer_scale_factor_nonlinearity_error_x = messageIn.accelerometer_scale_factor_nonlinearity_error_x;
	messageOut->accelerometer_scale_factor_nonlinearity_error_y = messageIn.accelerometer_scale_factor_nonlinearity_error_y;
	messageOut->accelerometer_scale_factor_nonlinearity_error_z = messageIn.accelerometer_scale_factor_nonlinearity_error_z;
}

// Topic to Msg_2411
void convert(hg_nav_node::Msg_2411 messageIn, Msg_2411 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gyro_bias_error_x = messageIn.gyro_bias_error_x;
	messageOut->gyro_bias_error_y = messageIn.gyro_bias_error_y;
	messageOut->gyro_bias_error_z = messageIn.gyro_bias_error_z;
	messageOut->gyro_bias_inrun_error_x = messageIn.gyro_bias_inrun_error_x;
	messageOut->gyro_bias_inrun_error_y = messageIn.gyro_bias_inrun_error_y;
	messageOut->gyro_bias_inrun_error_z = messageIn.gyro_bias_inrun_error_z;
	messageOut->gyro_scale_factor_error_x = messageIn.gyro_scale_factor_error_x;
	messageOut->gyro_scale_factor_error_y = messageIn.gyro_scale_factor_error_y;
	messageOut->gyro_scale_factor_error_z = messageIn.gyro_scale_factor_error_z;
	messageOut->gyro_nonorthogonality_error_yz = messageIn.gyro_nonorthogonality_error_yz;
	messageOut->gyro_nonorthogonality_error_zx = messageIn.gyro_nonorthogonality_error_zx;
	messageOut->gyro_nonorthogonality_error_xy = messageIn.gyro_nonorthogonality_error_xy;
	messageOut->accelerometer_bias_error_x = messageIn.accelerometer_bias_error_x;
	messageOut->accelerometer_bias_error_y = messageIn.accelerometer_bias_error_y;
	messageOut->accelerometer_bias_error_z = messageIn.accelerometer_bias_error_z;
	messageOut->accelerometer_bias_inrun_error_x = messageIn.accelerometer_bias_inrun_error_x;
	messageOut->accelerometer_bias_inrun_error_y = messageIn.accelerometer_bias_inrun_error_y;
	messageOut->accelerometer_bias_inrun_error_z = messageIn.accelerometer_bias_inrun_error_z;
	messageOut->accelerometer_scale_factor_error_x = messageIn.accelerometer_scale_factor_error_x;
	messageOut->accelerometer_scale_factor_error_y = messageIn.accelerometer_scale_factor_error_y;
	messageOut->accelerometer_scale_factor_error_z = messageIn.accelerometer_scale_factor_error_z;
	messageOut->accelerometer_nonorthogonality_error_yz = messageIn.accelerometer_nonorthogonality_error_yz;
	messageOut->accelerometer_nonorthogonality_error_zx = messageIn.accelerometer_nonorthogonality_error_zx;
	messageOut->accelerometer_nonorthogonality_error_xy = messageIn.accelerometer_nonorthogonality_error_xy;
	messageOut->accelerometer_misalignment_error_x = messageIn.accelerometer_misalignment_error_x;
	messageOut->accelerometer_misalignment_error_y = messageIn.accelerometer_misalignment_error_y;
	messageOut->accelerometer_misalignment_error_z = messageIn.accelerometer_misalignment_error_z;
	messageOut->accelerometer_scale_factor_nonlinearity_error_x = messageIn.accelerometer_scale_factor_nonlinearity_error_x;
	messageOut->accelerometer_scale_factor_nonlinearity_error_y = messageIn.accelerometer_scale_factor_nonlinearity_error_y;
	messageOut->accelerometer_scale_factor_nonlinearity_error_z = messageIn.accelerometer_scale_factor_nonlinearity_error_z;
}

void Msg_2411_pub_callback(uint8_t * buffer)
{
	Msg_2411 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2411 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2411);
	ROS_DEBUG("Message 0x2411 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2411_pub_initialized == false){
		init_2411(getRosHandle());}
	// Publish the message
	Msg_2411_pub.publish(msgStruct_2411);
	return;
}
