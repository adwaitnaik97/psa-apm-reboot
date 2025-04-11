#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_0C.h>
#include <hg_node/hg1120_multiplex_status_word2_t.h>
#include <hg_node/hg1120_status_word_1_t.h>
hg_node::Msg_0C msgStruct_0C;

bool Msg_0C_pub_initialized = false;

ros::Publisher Msg_0C_pub;
void init_0C(ros::NodeHandle * n){
	Msg_0C_pub = n->advertise<hg_node::Msg_0C>(MSG_0C_PATH, 5);
	Msg_0C_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_0C_PATH);
	return;
}

void stop_0C(void){
	if (Msg_0C_pub_initialized){
		Msg_0C_pub.shutdown();
		Msg_0C_pub_initialized = false;
		ROS_INFO("0x0C stopped");
	}
	return;
}

// Msg_0C to Topic
void convert(Msg_0C messageIn, hg_node::Msg_0C * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;

	messageOut->MainStatusWord.MuxStatusCounter = messageIn.MainStatusWord.MuxStatusCounter;
	messageOut->MainStatusWord.IMU_OK = messageIn.MainStatusWord.IMU_OK;
	messageOut->MainStatusWord.SensorBoardInit = messageIn.MainStatusWord.SensorBoardInit;
	messageOut->MainStatusWord.AccelXValidity = messageIn.MainStatusWord.AccelXValidity;
	messageOut->MainStatusWord.AccelYValidity = messageIn.MainStatusWord.AccelYValidity;
	messageOut->MainStatusWord.AccelZValidity = messageIn.MainStatusWord.AccelZValidity;
	messageOut->MainStatusWord.GyroXValidity = messageIn.MainStatusWord.GyroXValidity;
	messageOut->MainStatusWord.GyroYValidity = messageIn.MainStatusWord.GyroYValidity;
	messageOut->MainStatusWord.GyroZValidity = messageIn.MainStatusWord.GyroZValidity;
	messageOut->MainStatusWord.MagnetometerValidity = messageIn.MainStatusWord.MagnetometerValidity;
	messageOut->MainStatusWord.PowerUp_BIT = messageIn.MainStatusWord.PowerUp_BIT;
	messageOut->MainStatusWord.Continuous_BIT = messageIn.MainStatusWord.Continuous_BIT;
	messageOut->MainStatusWord.PowerUp_test = messageIn.MainStatusWord.PowerUp_test;

	messageOut->MultiPlexedStatusWord.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord.SensorElectronicsStatus = messageIn.MultiPlexedStatusWord.SensorElectronicsStatus;
	messageOut->MultiPlexedStatusWord.SensorDataReadyStatus = messageIn.MultiPlexedStatusWord.SensorDataReadyStatus;
	messageOut->MultiPlexedStatusWord.TemperatureStatus = messageIn.MultiPlexedStatusWord.TemperatureStatus;
	messageOut->MultiPlexedStatusWord.AccelerometerXHealth = messageIn.MultiPlexedStatusWord.AccelerometerXHealth;
	messageOut->MultiPlexedStatusWord.AccelerometerYHealth = messageIn.MultiPlexedStatusWord.AccelerometerYHealth;
	messageOut->MultiPlexedStatusWord.AccelerometerZHealth = messageIn.MultiPlexedStatusWord.AccelerometerZHealth;
	messageOut->MultiPlexedStatusWord.GyroXHealth = messageIn.MultiPlexedStatusWord.GyroXHealth;
	messageOut->MultiPlexedStatusWord.GyroYHealth = messageIn.MultiPlexedStatusWord.GyroYHealth;
	messageOut->MultiPlexedStatusWord.GyroZHealth = messageIn.MultiPlexedStatusWord.GyroZHealth;
	messageOut->MultiPlexedStatusWord.TemperatureStatusLatched = messageIn.MultiPlexedStatusWord.TemperatureStatusLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerXHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerXHealthLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerYHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerYHealthLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerZHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerZHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroXHealthLatched = messageIn.MultiPlexedStatusWord.GyroXHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroYHealthLatched = messageIn.MultiPlexedStatusWord.GyroYHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroZHealthLatched = messageIn.MultiPlexedStatusWord.GyroZHealthLatched;
	messageOut->MultiPlexedStatusWord.MagnetometerXHealth = messageIn.MultiPlexedStatusWord.MagnetometerXHealth;
	messageOut->MultiPlexedStatusWord.MagnetometerYHealth = messageIn.MultiPlexedStatusWord.MagnetometerYHealth;
	messageOut->MultiPlexedStatusWord.MagnetometerZHealth = messageIn.MultiPlexedStatusWord.MagnetometerZHealth;
	messageOut->MultiPlexedStatusWord.LoopCompletionTest = messageIn.MultiPlexedStatusWord.LoopCompletionTest;
	messageOut->MultiPlexedStatusWord.RAMTest = messageIn.MultiPlexedStatusWord.RAMTest;
	messageOut->MultiPlexedStatusWord.CoefficientTableCRCTest = messageIn.MultiPlexedStatusWord.CoefficientTableCRCTest;
	messageOut->MultiPlexedStatusWord.ConfigurationTableCRCTest = messageIn.MultiPlexedStatusWord.ConfigurationTableCRCTest;
	messageOut->MultiPlexedStatusWord.NormalModeSWCRCTest = messageIn.MultiPlexedStatusWord.NormalModeSWCRCTest;
	messageOut->MultiPlexedStatusWord.StackOverflowTest = messageIn.MultiPlexedStatusWord.StackOverflowTest;
	messageOut->MultiPlexedStatusWord.WatchdogTimerTest = messageIn.MultiPlexedStatusWord.WatchdogTimerTest;
	messageOut->MultiPlexedStatusWord.ProcessorTest = messageIn.MultiPlexedStatusWord.ProcessorTest;
	messageOut->MultiPlexedStatusWord.LoopCompletionTestLatched = messageIn.MultiPlexedStatusWord.LoopCompletionTestLatched;
	messageOut->MultiPlexedStatusWord.RAMTestLatched = messageIn.MultiPlexedStatusWord.RAMTestLatched;
	messageOut->MultiPlexedStatusWord.CoefficientTableCRCTestLatched = messageIn.MultiPlexedStatusWord.CoefficientTableCRCTestLatched;
	messageOut->MultiPlexedStatusWord.ConfigurationTableCRCTestLatched = messageIn.MultiPlexedStatusWord.ConfigurationTableCRCTestLatched;
	messageOut->MultiPlexedStatusWord.NormalModeSWCRCTestLatched = messageIn.MultiPlexedStatusWord.NormalModeSWCRCTestLatched;
	messageOut->MultiPlexedStatusWord.StackOverflowTestLatched = messageIn.MultiPlexedStatusWord.StackOverflowTestLatched;
	messageOut->MultiPlexedStatusWord.WatchdogTimerTestLatched = messageIn.MultiPlexedStatusWord.WatchdogTimerTestLatched;
	messageOut->MultiPlexedStatusWord.ProcessorTestLatched = messageIn.MultiPlexedStatusWord.ProcessorTestLatched;
	messageOut->MultiPlexedStatusWord.SensorTemperature = messageIn.MultiPlexedStatusWord.SensorTemperature;
	messageOut->MultiPlexedStatusWord.MagnetometerTemperature = messageIn.MultiPlexedStatusWord.MagnetometerTemperature;
	messageOut->MultiPlexedStatusWord.DIO1 = messageIn.MultiPlexedStatusWord.DIO1;
	messageOut->MultiPlexedStatusWord.DIO2 = messageIn.MultiPlexedStatusWord.DIO2;
	messageOut->MultiPlexedStatusWord.DIO3 = messageIn.MultiPlexedStatusWord.DIO3;
	messageOut->MultiPlexedStatusWord.DIO4 = messageIn.MultiPlexedStatusWord.DIO4;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_0C
void convert(hg_node::Msg_0C messageIn, Msg_0C * messageOut)
{
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;

	messageOut->MainStatusWord.MuxStatusCounter = messageIn.MainStatusWord.MuxStatusCounter;
	messageOut->MainStatusWord.IMU_OK = messageIn.MainStatusWord.IMU_OK;
	messageOut->MainStatusWord.SensorBoardInit = messageIn.MainStatusWord.SensorBoardInit;
	messageOut->MainStatusWord.AccelXValidity = messageIn.MainStatusWord.AccelXValidity;
	messageOut->MainStatusWord.AccelYValidity = messageIn.MainStatusWord.AccelYValidity;
	messageOut->MainStatusWord.AccelZValidity = messageIn.MainStatusWord.AccelZValidity;
	messageOut->MainStatusWord.GyroXValidity = messageIn.MainStatusWord.GyroXValidity;
	messageOut->MainStatusWord.GyroYValidity = messageIn.MainStatusWord.GyroYValidity;
	messageOut->MainStatusWord.GyroZValidity = messageIn.MainStatusWord.GyroZValidity;
	messageOut->MainStatusWord.MagnetometerValidity = messageIn.MainStatusWord.MagnetometerValidity;
	messageOut->MainStatusWord.PowerUp_BIT = messageIn.MainStatusWord.PowerUp_BIT;
	messageOut->MainStatusWord.Continuous_BIT = messageIn.MainStatusWord.Continuous_BIT;
	messageOut->MainStatusWord.PowerUp_test = messageIn.MainStatusWord.PowerUp_test;

	messageOut->MultiPlexedStatusWord.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord.SensorElectronicsStatus = messageIn.MultiPlexedStatusWord.SensorElectronicsStatus;
	messageOut->MultiPlexedStatusWord.SensorDataReadyStatus = messageIn.MultiPlexedStatusWord.SensorDataReadyStatus;
	messageOut->MultiPlexedStatusWord.TemperatureStatus = messageIn.MultiPlexedStatusWord.TemperatureStatus;
	messageOut->MultiPlexedStatusWord.AccelerometerXHealth = messageIn.MultiPlexedStatusWord.AccelerometerXHealth;
	messageOut->MultiPlexedStatusWord.AccelerometerYHealth = messageIn.MultiPlexedStatusWord.AccelerometerYHealth;
	messageOut->MultiPlexedStatusWord.AccelerometerZHealth = messageIn.MultiPlexedStatusWord.AccelerometerZHealth;
	messageOut->MultiPlexedStatusWord.GyroXHealth = messageIn.MultiPlexedStatusWord.GyroXHealth;
	messageOut->MultiPlexedStatusWord.GyroYHealth = messageIn.MultiPlexedStatusWord.GyroYHealth;
	messageOut->MultiPlexedStatusWord.GyroZHealth = messageIn.MultiPlexedStatusWord.GyroZHealth;
	messageOut->MultiPlexedStatusWord.TemperatureStatusLatched = messageIn.MultiPlexedStatusWord.TemperatureStatusLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerXHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerXHealthLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerYHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerYHealthLatched;
	messageOut->MultiPlexedStatusWord.AccelerometerZHealthLatched = messageIn.MultiPlexedStatusWord.AccelerometerZHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroXHealthLatched = messageIn.MultiPlexedStatusWord.GyroXHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroYHealthLatched = messageIn.MultiPlexedStatusWord.GyroYHealthLatched;
	messageOut->MultiPlexedStatusWord.GyroZHealthLatched = messageIn.MultiPlexedStatusWord.GyroZHealthLatched;
	messageOut->MultiPlexedStatusWord.MagnetometerXHealth = messageIn.MultiPlexedStatusWord.MagnetometerXHealth;
	messageOut->MultiPlexedStatusWord.MagnetometerYHealth = messageIn.MultiPlexedStatusWord.MagnetometerYHealth;
	messageOut->MultiPlexedStatusWord.MagnetometerZHealth = messageIn.MultiPlexedStatusWord.MagnetometerZHealth;
	messageOut->MultiPlexedStatusWord.LoopCompletionTest = messageIn.MultiPlexedStatusWord.LoopCompletionTest;
	messageOut->MultiPlexedStatusWord.RAMTest = messageIn.MultiPlexedStatusWord.RAMTest;
	messageOut->MultiPlexedStatusWord.CoefficientTableCRCTest = messageIn.MultiPlexedStatusWord.CoefficientTableCRCTest;
	messageOut->MultiPlexedStatusWord.ConfigurationTableCRCTest = messageIn.MultiPlexedStatusWord.ConfigurationTableCRCTest;
	messageOut->MultiPlexedStatusWord.NormalModeSWCRCTest = messageIn.MultiPlexedStatusWord.NormalModeSWCRCTest;
	messageOut->MultiPlexedStatusWord.StackOverflowTest = messageIn.MultiPlexedStatusWord.StackOverflowTest;
	messageOut->MultiPlexedStatusWord.WatchdogTimerTest = messageIn.MultiPlexedStatusWord.WatchdogTimerTest;
	messageOut->MultiPlexedStatusWord.ProcessorTest = messageIn.MultiPlexedStatusWord.ProcessorTest;
	messageOut->MultiPlexedStatusWord.LoopCompletionTestLatched = messageIn.MultiPlexedStatusWord.LoopCompletionTestLatched;
	messageOut->MultiPlexedStatusWord.RAMTestLatched = messageIn.MultiPlexedStatusWord.RAMTestLatched;
	messageOut->MultiPlexedStatusWord.CoefficientTableCRCTestLatched = messageIn.MultiPlexedStatusWord.CoefficientTableCRCTestLatched;
	messageOut->MultiPlexedStatusWord.ConfigurationTableCRCTestLatched = messageIn.MultiPlexedStatusWord.ConfigurationTableCRCTestLatched;
	messageOut->MultiPlexedStatusWord.NormalModeSWCRCTestLatched = messageIn.MultiPlexedStatusWord.NormalModeSWCRCTestLatched;
	messageOut->MultiPlexedStatusWord.StackOverflowTestLatched = messageIn.MultiPlexedStatusWord.StackOverflowTestLatched;
	messageOut->MultiPlexedStatusWord.WatchdogTimerTestLatched = messageIn.MultiPlexedStatusWord.WatchdogTimerTestLatched;
	messageOut->MultiPlexedStatusWord.ProcessorTestLatched = messageIn.MultiPlexedStatusWord.ProcessorTestLatched;
	messageOut->MultiPlexedStatusWord.SensorTemperature = messageIn.MultiPlexedStatusWord.SensorTemperature;
	messageOut->MultiPlexedStatusWord.MagnetometerTemperature = messageIn.MultiPlexedStatusWord.MagnetometerTemperature;
	messageOut->MultiPlexedStatusWord.DIO1 = messageIn.MultiPlexedStatusWord.DIO1;
	messageOut->MultiPlexedStatusWord.DIO2 = messageIn.MultiPlexedStatusWord.DIO2;
	messageOut->MultiPlexedStatusWord.DIO3 = messageIn.MultiPlexedStatusWord.DIO3;
	messageOut->MultiPlexedStatusWord.DIO4 = messageIn.MultiPlexedStatusWord.DIO4;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_0C_pub_callback(uint8_t * buffer)
{
	Msg_0C Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x0C deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_0C);
	ROS_DEBUG("Message 0x0C Received");

	// Initialize Publisher if not initialized yet
	if (Msg_0C_pub_initialized == false){
		init_0C(getRosHandle());}
	// Publish the message
	Msg_0C_pub.publish(msgStruct_0C);
	return;
}
