#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_CA.h>
#include <hg_node/hg4930_multiplex_status_word2_t.h>
#include <hg_node/hg4930_status_word_1_t.h>
hg_node::Msg_CA msgStruct_CA;

bool Msg_CA_pub_initialized = false;

ros::Publisher Msg_CA_pub;
void init_CA(ros::NodeHandle * n){
	Msg_CA_pub = n->advertise<hg_node::Msg_CA>(MSG_CA_PATH, 5);
	Msg_CA_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_CA_PATH);
	return;
}

void stop_CA(void){
	if (Msg_CA_pub_initialized){
		Msg_CA_pub.shutdown();
		Msg_CA_pub_initialized = false;
		ROS_INFO("0xCA stopped");
	}
	return;
}

// Msg_CA to Topic
void convert(Msg_CA messageIn, hg_node::Msg_CA * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;

	messageOut->StatusWord1.Counter = messageIn.StatusWord1.Counter;
	messageOut->StatusWord1.BITmodeIndicator = messageIn.StatusWord1.BITmodeIndicator;
	messageOut->StatusWord1.IMU_BIT_Summary = messageIn.StatusWord1.IMU_BIT_Summary;
	messageOut->StatusWord1.Gyro_BIT_Summary = messageIn.StatusWord1.Gyro_BIT_Summary;
	messageOut->StatusWord1.Accel_BIT_Summary = messageIn.StatusWord1.Accel_BIT_Summary;
	messageOut->StatusWord1.GyroVoltage_BIT = messageIn.StatusWord1.GyroVoltage_BIT;
	messageOut->StatusWord1.GyroX_BIT = messageIn.StatusWord1.GyroX_BIT;
	messageOut->StatusWord1.GyroY_BIT = messageIn.StatusWord1.GyroY_BIT;
	messageOut->StatusWord1.GyroZ_BIT = messageIn.StatusWord1.GyroZ_BIT;
	messageOut->StatusWord1.IMU_OK = messageIn.StatusWord1.IMU_OK;

	messageOut->MultiPlexedStatusWord2.GyroHealth1 = messageIn.MultiPlexedStatusWord2.GyroHealth1;
	messageOut->MultiPlexedStatusWord2.StartDataFlag = messageIn.MultiPlexedStatusWord2.StartDataFlag;
	messageOut->MultiPlexedStatusWord2.ProcessTest = messageIn.MultiPlexedStatusWord2.ProcessTest;
	messageOut->MultiPlexedStatusWord2.MemoryTest = messageIn.MultiPlexedStatusWord2.MemoryTest;
	messageOut->MultiPlexedStatusWord2.ElectronicsTest = messageIn.MultiPlexedStatusWord2.ElectronicsTest;
	messageOut->MultiPlexedStatusWord2.GyroHealth2 = messageIn.MultiPlexedStatusWord2.GyroHealth2;
	messageOut->MultiPlexedStatusWord2.AcceHealth = messageIn.MultiPlexedStatusWord2.AcceHealth;
	messageOut->MultiPlexedStatusWord2.StatusWord2ID = messageIn.MultiPlexedStatusWord2.StatusWord2ID;
	messageOut->MultiPlexedStatusWord2.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord2.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord2.AccelXTemperature = messageIn.MultiPlexedStatusWord2.AccelXTemperature;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_CA
void convert(hg_node::Msg_CA messageIn, Msg_CA * messageOut)
{
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;

	messageOut->StatusWord1.Counter = messageIn.StatusWord1.Counter;
	messageOut->StatusWord1.BITmodeIndicator = messageIn.StatusWord1.BITmodeIndicator;
	messageOut->StatusWord1.IMU_BIT_Summary = messageIn.StatusWord1.IMU_BIT_Summary;
	messageOut->StatusWord1.Gyro_BIT_Summary = messageIn.StatusWord1.Gyro_BIT_Summary;
	messageOut->StatusWord1.Accel_BIT_Summary = messageIn.StatusWord1.Accel_BIT_Summary;
	messageOut->StatusWord1.GyroVoltage_BIT = messageIn.StatusWord1.GyroVoltage_BIT;
	messageOut->StatusWord1.GyroX_BIT = messageIn.StatusWord1.GyroX_BIT;
	messageOut->StatusWord1.GyroY_BIT = messageIn.StatusWord1.GyroY_BIT;
	messageOut->StatusWord1.GyroZ_BIT = messageIn.StatusWord1.GyroZ_BIT;
	messageOut->StatusWord1.IMU_OK = messageIn.StatusWord1.IMU_OK;

	messageOut->MultiPlexedStatusWord2.GyroHealth1 = messageIn.MultiPlexedStatusWord2.GyroHealth1;
	messageOut->MultiPlexedStatusWord2.StartDataFlag = messageIn.MultiPlexedStatusWord2.StartDataFlag;
	messageOut->MultiPlexedStatusWord2.ProcessTest = messageIn.MultiPlexedStatusWord2.ProcessTest;
	messageOut->MultiPlexedStatusWord2.MemoryTest = messageIn.MultiPlexedStatusWord2.MemoryTest;
	messageOut->MultiPlexedStatusWord2.ElectronicsTest = messageIn.MultiPlexedStatusWord2.ElectronicsTest;
	messageOut->MultiPlexedStatusWord2.GyroHealth2 = messageIn.MultiPlexedStatusWord2.GyroHealth2;
	messageOut->MultiPlexedStatusWord2.AcceHealth = messageIn.MultiPlexedStatusWord2.AcceHealth;
	messageOut->MultiPlexedStatusWord2.StatusWord2ID = messageIn.MultiPlexedStatusWord2.StatusWord2ID;
	messageOut->MultiPlexedStatusWord2.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord2.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord2.AccelXTemperature = messageIn.MultiPlexedStatusWord2.AccelXTemperature;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_CA_pub_callback(uint8_t * buffer)
{
	Msg_CA Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xCA deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_CA);
	ROS_DEBUG("Message 0xCA Received");

	// Initialize Publisher if not initialized yet
	if (Msg_CA_pub_initialized == false){
		init_CA(getRosHandle());}
	// Publish the message
	Msg_CA_pub.publish(msgStruct_CA);
	return;
}
