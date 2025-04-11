#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_B0.h>
#include <hg_node/filter_config_t.h>
#include <hg_node/sensor_axes_t.h>
hg_node::Msg_B0 msgStruct_B0;

bool Msg_B0_pub_initialized = false;

ros::Publisher Msg_B0_pub;
void init_B0(ros::NodeHandle * n){
	Msg_B0_pub = n->advertise<hg_node::Msg_B0>(MSG_B0_PATH, 5);
	Msg_B0_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_B0_PATH);
	return;
}

void stop_B0(void){
	if (Msg_B0_pub_initialized){
		Msg_B0_pub.shutdown();
		Msg_B0_pub_initialized = false;
		ROS_INFO("0xB0 stopped");
	}
	return;
}

// Msg_B0 to Topic
void convert(Msg_B0 messageIn, hg_node::Msg_B0 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->fw_major = messageIn.fw_major;
	messageOut->fw_minor = messageIn.fw_minor;
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_type[index] = messageIn.device_type[index];
	}

	messageOut->part_number = messageIn.part_number;
	messageOut->hardware_version = messageIn.hardware_version;
	messageOut->device_config = messageIn.device_config;
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->serial_number[index] = messageIn.serial_number[index];
	}


	messageOut->gyro_filter_config.cutoff_frequency = messageIn.gyro_filter_config.cutoff_frequency;
	messageOut->gyro_filter_config.enabled = messageIn.gyro_filter_config.enabled;

	messageOut->accel_filter_config.cutoff_frequency = messageIn.accel_filter_config.cutoff_frequency;
	messageOut->accel_filter_config.enabled = messageIn.accel_filter_config.enabled;
	messageOut->gyro_range = messageIn.gyro_range;
	messageOut->accel_range = messageIn.accel_range;
	messageOut->hours_of_operation = messageIn.hours_of_operation;
	messageOut->boot_count = messageIn.boot_count;
	messageOut->baud_rate = messageIn.baud_rate;
	messageOut->sampling_frequency = messageIn.sampling_frequency;
	messageOut->bytes_transmitted = messageIn.bytes_transmitted;
	messageOut->percent_fr = messageIn.percent_fr;
	messageOut->percent_ffr = messageIn.percent_ffr;
	messageOut->percent_cr = messageIn.percent_cr;
	messageOut->percent_gr = messageIn.percent_gr;
	messageOut->percent_100_fr = messageIn.percent_100_fr;
	messageOut->percent_10_fr = messageIn.percent_10_fr;
	messageOut->percent_1_fr = messageIn.percent_1_fr;
	messageOut->device_current = messageIn.device_current;
	messageOut->device_voltage = messageIn.device_voltage;

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
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_B0
void convert(hg_node::Msg_B0 messageIn, Msg_B0 * messageOut)
{
	messageOut->fw_major = messageIn.fw_major;
	messageOut->fw_minor = messageIn.fw_minor;
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->device_type[index] = messageIn.device_type[index];
	}

	messageOut->part_number = messageIn.part_number;
	messageOut->hardware_version = messageIn.hardware_version;
	messageOut->device_config = messageIn.device_config;
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->serial_number[index] = messageIn.serial_number[index];
	}


	messageOut->gyro_filter_config.cutoff_frequency = messageIn.gyro_filter_config.cutoff_frequency;
	messageOut->gyro_filter_config.enabled = messageIn.gyro_filter_config.enabled;

	messageOut->accel_filter_config.cutoff_frequency = messageIn.accel_filter_config.cutoff_frequency;
	messageOut->accel_filter_config.enabled = messageIn.accel_filter_config.enabled;
	messageOut->gyro_range = messageIn.gyro_range;
	messageOut->accel_range = messageIn.accel_range;
	messageOut->hours_of_operation = messageIn.hours_of_operation;
	messageOut->boot_count = messageIn.boot_count;
	messageOut->baud_rate = messageIn.baud_rate;
	messageOut->sampling_frequency = messageIn.sampling_frequency;
	messageOut->bytes_transmitted = messageIn.bytes_transmitted;
	messageOut->percent_fr = messageIn.percent_fr;
	messageOut->percent_ffr = messageIn.percent_ffr;
	messageOut->percent_cr = messageIn.percent_cr;
	messageOut->percent_gr = messageIn.percent_gr;
	messageOut->percent_100_fr = messageIn.percent_100_fr;
	messageOut->percent_10_fr = messageIn.percent_10_fr;
	messageOut->percent_1_fr = messageIn.percent_1_fr;
	messageOut->device_current = messageIn.device_current;
	messageOut->device_voltage = messageIn.device_voltage;

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
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_B0_pub_callback(uint8_t * buffer)
{
	Msg_B0 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xB0 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_B0);
	ROS_DEBUG("Message 0xB0 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_B0_pub_initialized == false){
		init_B0(getRosHandle());}
	// Publish the message
	Msg_B0_pub.publish(msgStruct_B0);
	return;
}
