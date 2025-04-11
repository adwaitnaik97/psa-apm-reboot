#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_B3.h>
#include <hg_node/status_word_t.h>
hg_node::Msg_B3 msgStruct_B3;

bool Msg_B3_pub_initialized = false;

ros::Publisher Msg_B3_pub;
void init_B3(ros::NodeHandle * n){
	Msg_B3_pub = n->advertise<hg_node::Msg_B3>(MSG_B3_PATH, 5);
	Msg_B3_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_B3_PATH);
	return;
}

void stop_B3(void){
	if (Msg_B3_pub_initialized){
		Msg_B3_pub.shutdown();
		Msg_B3_pub_initialized = false;
		ROS_INFO("0xB3 stopped");
	}
	return;
}

// Msg_B3 to Topic
void convert(Msg_B3 messageIn, hg_node::Msg_B3 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;
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

// Topic to Msg_B3
void convert(hg_node::Msg_B3 messageIn, Msg_B3 * messageOut)
{
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;
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

void Msg_B3_pub_callback(uint8_t * buffer)
{
	Msg_B3 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xB3 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_B3);
	ROS_DEBUG("Message 0xB3 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_B3_pub_initialized == false){
		init_B3(getRosHandle());}
	// Publish the message
	Msg_B3_pub.publish(msgStruct_B3);
	return;
}
