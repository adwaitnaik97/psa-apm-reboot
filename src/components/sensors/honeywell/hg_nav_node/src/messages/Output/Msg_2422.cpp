#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2422.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2422 msgStruct_2422;

bool Msg_2422_pub_initialized = false;

ros::Publisher Msg_2422_pub;
void init_2422(ros::NodeHandle * n){
	Msg_2422_pub = n->advertise<hg_nav_node::Msg_2422>(MSG_2422_PATH, 5);
	Msg_2422_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2422_PATH);
	return;
}

void stop_2422(void){
	Msg_2422_pub.shutdown();
	Msg_2422_pub_initialized = false;
	ROS_INFO("0x2422 stopped");
	return;
}

// Msg_2422 to Topic
void convert(Msg_2422 messageIn, hg_nav_node::Msg_2422 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->ecef_pos_x_std_dev = messageIn.ecef_pos_x_std_dev;
	messageOut->ecef_pos_y_std_dev = messageIn.ecef_pos_y_std_dev;
	messageOut->ecef_pos_z_std_dev = messageIn.ecef_pos_z_std_dev;
	messageOut->ecef_vel_x_std_dev = messageIn.ecef_vel_x_std_dev;
	messageOut->ecef_vel_y_std_dev = messageIn.ecef_vel_y_std_dev;
	messageOut->ecef_vel_z_std_dev = messageIn.ecef_vel_z_std_dev;
	messageOut->attitude_roll_std_dev = messageIn.attitude_roll_std_dev;
	messageOut->attitude_pitch_std_dev = messageIn.attitude_pitch_std_dev;
	messageOut->attitude_true_heading_std_dev = messageIn.attitude_true_heading_std_dev;
	messageOut->vehicle_body_x_axis_rotational_angle_std_dev = messageIn.vehicle_body_x_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_y_axis_rotational_angle_std_dev = messageIn.vehicle_body_y_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_z_axis_rotational_angle_std_dev = messageIn.vehicle_body_z_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_x_axis_linear_acceleration_std_dev = messageIn.vehicle_body_x_axis_linear_acceleration_std_dev;
	messageOut->vehicle_body_y_axis_linear_acceleration_std_dev = messageIn.vehicle_body_y_axis_linear_acceleration_std_dev;
	messageOut->vehicle_body_z_axis_linear_acceleration_std_dev = messageIn.vehicle_body_z_axis_linear_acceleration_std_dev;
	messageOut->zero_velocity_x_norm_meas_resid = messageIn.zero_velocity_x_norm_meas_resid;
	messageOut->zero_velocity_y_norm_meas_resid = messageIn.zero_velocity_y_norm_meas_resid;
	messageOut->zero_velocity_z_norm_meas_resid = messageIn.zero_velocity_z_norm_meas_resid;
	messageOut->zero_heading_change_norm_meas_resid = messageIn.zero_heading_change_norm_meas_resid;
}

// Topic to Msg_2422
void convert(hg_nav_node::Msg_2422 messageIn, Msg_2422 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->ecef_pos_x_std_dev = messageIn.ecef_pos_x_std_dev;
	messageOut->ecef_pos_y_std_dev = messageIn.ecef_pos_y_std_dev;
	messageOut->ecef_pos_z_std_dev = messageIn.ecef_pos_z_std_dev;
	messageOut->ecef_vel_x_std_dev = messageIn.ecef_vel_x_std_dev;
	messageOut->ecef_vel_y_std_dev = messageIn.ecef_vel_y_std_dev;
	messageOut->ecef_vel_z_std_dev = messageIn.ecef_vel_z_std_dev;
	messageOut->attitude_roll_std_dev = messageIn.attitude_roll_std_dev;
	messageOut->attitude_pitch_std_dev = messageIn.attitude_pitch_std_dev;
	messageOut->attitude_true_heading_std_dev = messageIn.attitude_true_heading_std_dev;
	messageOut->vehicle_body_x_axis_rotational_angle_std_dev = messageIn.vehicle_body_x_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_y_axis_rotational_angle_std_dev = messageIn.vehicle_body_y_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_z_axis_rotational_angle_std_dev = messageIn.vehicle_body_z_axis_rotational_angle_std_dev;
	messageOut->vehicle_body_x_axis_linear_acceleration_std_dev = messageIn.vehicle_body_x_axis_linear_acceleration_std_dev;
	messageOut->vehicle_body_y_axis_linear_acceleration_std_dev = messageIn.vehicle_body_y_axis_linear_acceleration_std_dev;
	messageOut->vehicle_body_z_axis_linear_acceleration_std_dev = messageIn.vehicle_body_z_axis_linear_acceleration_std_dev;
	messageOut->zero_velocity_x_norm_meas_resid = messageIn.zero_velocity_x_norm_meas_resid;
	messageOut->zero_velocity_y_norm_meas_resid = messageIn.zero_velocity_y_norm_meas_resid;
	messageOut->zero_velocity_z_norm_meas_resid = messageIn.zero_velocity_z_norm_meas_resid;
	messageOut->zero_heading_change_norm_meas_resid = messageIn.zero_heading_change_norm_meas_resid;
}

void Msg_2422_pub_callback(uint8_t * buffer)
{
	Msg_2422 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2422 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2422);
	ROS_DEBUG("Message 0x2422 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2422_pub_initialized == false){
		init_2422(getRosHandle());}
	// Publish the message
	Msg_2422_pub.publish(msgStruct_2422);
	return;
}
