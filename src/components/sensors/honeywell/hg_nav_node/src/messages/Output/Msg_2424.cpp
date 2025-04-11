#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2424.h>
#include <hg_nav_node/ins_mode_table_t.h>
hg_nav_node::Msg_2424 msgStruct_2424;

bool Msg_2424_pub_initialized = false;

ros::Publisher Msg_2424_pub;
void init_2424(ros::NodeHandle * n){
	Msg_2424_pub = n->advertise<hg_nav_node::Msg_2424>(MSG_2424_PATH, 5);
	Msg_2424_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2424_PATH);
	return;
}

void stop_2424(void){
	Msg_2424_pub.shutdown();
	Msg_2424_pub_initialized = false;
	ROS_INFO("0x2424 stopped");
	return;
}

// Msg_2424 to Topic
void convert(Msg_2424 messageIn, hg_nav_node::Msg_2424 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode.value = static_cast<uint8_t>(messageIn.INSMode);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gps_clk_phase_error = messageIn.gps_clk_phase_error;
	messageOut->gps_clk_freq_error = messageIn.gps_clk_freq_error;
	messageOut->gps_clk_acc_error = messageIn.gps_clk_acc_error;
	messageOut->gps_clk_g_sensitivity_x = messageIn.gps_clk_g_sensitivity_x;
	messageOut->gps_clk_g_sensitivity_y = messageIn.gps_clk_g_sensitivity_y;
	messageOut->gps_clk_g_sensitivity_z = messageIn.gps_clk_g_sensitivity_z;
	messageOut->min_meas_range_bias = messageIn.min_meas_range_bias;
	messageOut->max_meas_range_bias = messageIn.max_meas_range_bias;
	messageOut->gps_los_lever_arm_x = messageIn.gps_los_lever_arm_x;
	messageOut->gps_los_lever_arm_y = messageIn.gps_los_lever_arm_y;
	messageOut->gps_los_lever_arm_z = messageIn.gps_los_lever_arm_z;
	messageOut->gps_pvt_lever_arm_x = messageIn.gps_pvt_lever_arm_x;
	messageOut->gps_pvt_lever_arm_y = messageIn.gps_pvt_lever_arm_y;
	messageOut->gps_pvt_lever_arm_z = messageIn.gps_pvt_lever_arm_z;
	messageOut->gps_position_state_x = messageIn.gps_position_state_x;
	messageOut->gps_position_state_y = messageIn.gps_position_state_y;
	messageOut->gps_position_state_z = messageIn.gps_position_state_z;
	messageOut->gps_clk_g_sensitivity_x_stdv = messageIn.gps_clk_g_sensitivity_x_stdv;
	messageOut->gps_clk_g_sensitivity_y_stdv = messageIn.gps_clk_g_sensitivity_y_stdv;
	messageOut->gps_clk_g_sensitivity_z_stdv = messageIn.gps_clk_g_sensitivity_z_stdv;
	messageOut->min_meas_range_bias_stdv = messageIn.min_meas_range_bias_stdv;
	messageOut->max_meas_range_bias_stdv = messageIn.max_meas_range_bias_stdv;
	messageOut->gps_los_lever_arm_x_stdv = messageIn.gps_los_lever_arm_x_stdv;
	messageOut->gps_los_lever_arm_y_stdv = messageIn.gps_los_lever_arm_y_stdv;
	messageOut->gps_los_lever_arm_z_stdv = messageIn.gps_los_lever_arm_z_stdv;
	messageOut->gps_pvt_lever_arm_x_stdv = messageIn.gps_pvt_lever_arm_x_stdv;
	messageOut->gps_pvt_lever_arm_y_stdv = messageIn.gps_pvt_lever_arm_y_stdv;
	messageOut->gps_pvt_lever_arm_z_stdv = messageIn.gps_pvt_lever_arm_z_stdv;
	messageOut->gps_position_state_stdv_x = messageIn.gps_position_state_stdv_x;
	messageOut->gps_position_state_stdv_y = messageIn.gps_position_state_stdv_y;
	messageOut->gps_position_state_stdv_z = messageIn.gps_position_state_stdv_z;
}

// Topic to Msg_2424
void convert(hg_nav_node::Msg_2424 messageIn, Msg_2424 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->INSMode = static_cast<ins_mode_table_t>(messageIn.INSMode.value);
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gps_clk_phase_error = messageIn.gps_clk_phase_error;
	messageOut->gps_clk_freq_error = messageIn.gps_clk_freq_error;
	messageOut->gps_clk_acc_error = messageIn.gps_clk_acc_error;
	messageOut->gps_clk_g_sensitivity_x = messageIn.gps_clk_g_sensitivity_x;
	messageOut->gps_clk_g_sensitivity_y = messageIn.gps_clk_g_sensitivity_y;
	messageOut->gps_clk_g_sensitivity_z = messageIn.gps_clk_g_sensitivity_z;
	messageOut->min_meas_range_bias = messageIn.min_meas_range_bias;
	messageOut->max_meas_range_bias = messageIn.max_meas_range_bias;
	messageOut->gps_los_lever_arm_x = messageIn.gps_los_lever_arm_x;
	messageOut->gps_los_lever_arm_y = messageIn.gps_los_lever_arm_y;
	messageOut->gps_los_lever_arm_z = messageIn.gps_los_lever_arm_z;
	messageOut->gps_pvt_lever_arm_x = messageIn.gps_pvt_lever_arm_x;
	messageOut->gps_pvt_lever_arm_y = messageIn.gps_pvt_lever_arm_y;
	messageOut->gps_pvt_lever_arm_z = messageIn.gps_pvt_lever_arm_z;
	messageOut->gps_position_state_x = messageIn.gps_position_state_x;
	messageOut->gps_position_state_y = messageIn.gps_position_state_y;
	messageOut->gps_position_state_z = messageIn.gps_position_state_z;
	messageOut->gps_clk_g_sensitivity_x_stdv = messageIn.gps_clk_g_sensitivity_x_stdv;
	messageOut->gps_clk_g_sensitivity_y_stdv = messageIn.gps_clk_g_sensitivity_y_stdv;
	messageOut->gps_clk_g_sensitivity_z_stdv = messageIn.gps_clk_g_sensitivity_z_stdv;
	messageOut->min_meas_range_bias_stdv = messageIn.min_meas_range_bias_stdv;
	messageOut->max_meas_range_bias_stdv = messageIn.max_meas_range_bias_stdv;
	messageOut->gps_los_lever_arm_x_stdv = messageIn.gps_los_lever_arm_x_stdv;
	messageOut->gps_los_lever_arm_y_stdv = messageIn.gps_los_lever_arm_y_stdv;
	messageOut->gps_los_lever_arm_z_stdv = messageIn.gps_los_lever_arm_z_stdv;
	messageOut->gps_pvt_lever_arm_x_stdv = messageIn.gps_pvt_lever_arm_x_stdv;
	messageOut->gps_pvt_lever_arm_y_stdv = messageIn.gps_pvt_lever_arm_y_stdv;
	messageOut->gps_pvt_lever_arm_z_stdv = messageIn.gps_pvt_lever_arm_z_stdv;
	messageOut->gps_position_state_stdv_x = messageIn.gps_position_state_stdv_x;
	messageOut->gps_position_state_stdv_y = messageIn.gps_position_state_stdv_y;
	messageOut->gps_position_state_stdv_z = messageIn.gps_position_state_stdv_z;
}

void Msg_2424_pub_callback(uint8_t * buffer)
{
	Msg_2424 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2424 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2424);
	ROS_DEBUG("Message 0x2424 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2424_pub_initialized == false){
		init_2424(getRosHandle());}
	// Publish the message
	Msg_2424_pub.publish(msgStruct_2424);
	return;
}
