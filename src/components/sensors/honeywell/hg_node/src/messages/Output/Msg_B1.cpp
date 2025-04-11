#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_B1.h>
#include <hg_node/status_word_t.h>
hg_node::Msg_B1 msgStruct_B1;

bool Msg_B1_pub_initialized = false;

ros::Publisher Msg_B1_pub;
void init_B1(ros::NodeHandle * n){
	Msg_B1_pub = n->advertise<hg_node::Msg_B1>(MSG_B1_PATH, 5);
	Msg_B1_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_B1_PATH);
	return;
}

void stop_B1(void){
	if (Msg_B1_pub_initialized){
		Msg_B1_pub.shutdown();
		Msg_B1_pub_initialized = false;
		ROS_INFO("0xB1 stopped");
	}
	return;
}

// Msg_B1 to Topic
void convert(Msg_B1 messageIn, hg_node::Msg_B1 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;
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

// Topic to Msg_B1
void convert(hg_node::Msg_B1 messageIn, Msg_B1 * messageOut)
{
	messageOut->session = messageIn.session;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;
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

void Msg_B1_pub_callback(uint8_t * buffer)
{
	Msg_B1 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xB1 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_B1);
	ROS_DEBUG("Message 0xB1 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_B1_pub_initialized == false){
		init_B1(getRosHandle());}
	// Publish the message
	Msg_B1_pub.publish(msgStruct_B1);
	return;
}
