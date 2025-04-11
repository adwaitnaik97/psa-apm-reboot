#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_05.h>
#include <hg_node/hg1120_multiplex_status_word2_t.h>
#include <hg_node/hg1120_status_word_1_t.h>
hg_node::Msg_05 msgStruct_05;

bool Msg_05_pub_initialized = false;

ros::Publisher Msg_05_pub;
void init_05(ros::NodeHandle * n){
	Msg_05_pub = n->advertise<hg_node::Msg_05>(MSG_05_PATH, 5);
	Msg_05_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_05_PATH);
	return;
}

void stop_05(void){
	if (Msg_05_pub_initialized){
		Msg_05_pub.shutdown();
		Msg_05_pub_initialized = false;
		ROS_INFO("0x05 stopped");
	}
	return;
}

// Msg_05 to Topic
void convert(Msg_05 messageIn, hg_node::Msg_05 * messageOut)
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
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_05
void convert(hg_node::Msg_05 messageIn, Msg_05 * messageOut)
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
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_05_pub_callback(uint8_t * buffer)
{
	Msg_05 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x05 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_05);
	ROS_DEBUG("Message 0x05 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_05_pub_initialized == false){
		init_05(getRosHandle());}
	// Publish the message
	Msg_05_pub.publish(msgStruct_05);
	return;
}
