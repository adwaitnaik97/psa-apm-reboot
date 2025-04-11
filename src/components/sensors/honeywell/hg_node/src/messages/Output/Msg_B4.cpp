#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_B4.h>
#include <hg_node/status_word_t.h>
hg_node::Msg_B4 msgStruct_B4;

bool Msg_B4_pub_initialized = false;

ros::Publisher Msg_B4_pub;
void init_B4(ros::NodeHandle * n){
	Msg_B4_pub = n->advertise<hg_node::Msg_B4>(MSG_B4_PATH, 5);
	Msg_B4_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_B4_PATH);
	return;
}

void stop_B4(void){
	if (Msg_B4_pub_initialized){
		Msg_B4_pub.shutdown();
		Msg_B4_pub_initialized = false;
		ROS_INFO("0xB4 stopped");
	}
	return;
}

// Msg_B4 to Topic
void convert(Msg_B4 messageIn, hg_node::Msg_B4 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->attitude_i = messageIn.attitude_i;
	messageOut->attitude_j = messageIn.attitude_j;
	messageOut->attitude_k = messageIn.attitude_k;
	messageOut->attitude_s = messageIn.attitude_s;
	messageOut->temperature = messageIn.temperature;

	messageOut->status_word.gyro_x_fail = messageIn.status_word.gyro_x_fail;
	messageOut->status_word.gyro_y_fail = messageIn.status_word.gyro_y_fail;
	messageOut->status_word.gyro_z_fail = messageIn.status_word.gyro_z_fail;
	messageOut->status_word.accel_x_fail = messageIn.status_word.accel_x_fail;
	messageOut->status_word.accel_y_fail = messageIn.status_word.accel_y_fail;
	messageOut->status_word.accel_z_fail = messageIn.status_word.accel_z_fail;
	messageOut->status_word.mag_x_fail = messageIn.status_word.mag_x_fail;
	messageOut->status_word.mag_y_fail = messageIn.status_word.mag_y_fail;
	messageOut->status_word.mag_z_fail = messageIn.status_word.mag_z_fail;
	messageOut->status_word.counter = messageIn.status_word.counter;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_B4
void convert(hg_node::Msg_B4 messageIn, Msg_B4 * messageOut)
{
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->attitude_i = messageIn.attitude_i;
	messageOut->attitude_j = messageIn.attitude_j;
	messageOut->attitude_k = messageIn.attitude_k;
	messageOut->attitude_s = messageIn.attitude_s;
	messageOut->temperature = messageIn.temperature;

	messageOut->status_word.gyro_x_fail = messageIn.status_word.gyro_x_fail;
	messageOut->status_word.gyro_y_fail = messageIn.status_word.gyro_y_fail;
	messageOut->status_word.gyro_z_fail = messageIn.status_word.gyro_z_fail;
	messageOut->status_word.accel_x_fail = messageIn.status_word.accel_x_fail;
	messageOut->status_word.accel_y_fail = messageIn.status_word.accel_y_fail;
	messageOut->status_word.accel_z_fail = messageIn.status_word.accel_z_fail;
	messageOut->status_word.mag_x_fail = messageIn.status_word.mag_x_fail;
	messageOut->status_word.mag_y_fail = messageIn.status_word.mag_y_fail;
	messageOut->status_word.mag_z_fail = messageIn.status_word.mag_z_fail;
	messageOut->status_word.counter = messageIn.status_word.counter;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_B4_pub_callback(uint8_t * buffer)
{
	Msg_B4 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xB4 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_B4);
	ROS_DEBUG("Message 0xB4 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_B4_pub_initialized == false){
		init_B4(getRosHandle());}
	// Publish the message
	Msg_B4_pub.publish(msgStruct_B4);
	return;
}
