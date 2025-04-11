#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_AE.h>
#include <hg_node/control_frequency_t.h>
#include <hg_node/guidance_frequency_t.h>
#include <hg_node/hgimu_multiplex_status_word2_t.h>
#include <hg_node/hgimu_status_word_1_t.h>
hg_node::Msg_AE msgStruct_AE;

bool Msg_AE_pub_initialized = false;

ros::Publisher Msg_AE_pub;
void init_AE(ros::NodeHandle * n){
	Msg_AE_pub = n->advertise<hg_node::Msg_AE>(MSG_AE_PATH, 5);
	Msg_AE_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_AE_PATH);
	return;
}

void stop_AE(void){
	if (Msg_AE_pub_initialized){
		Msg_AE_pub.shutdown();
		Msg_AE_pub_initialized = false;
		ROS_INFO("0xAE stopped");
	}
	return;
}

// Msg_AE to Topic
void convert(Msg_AE messageIn, hg_node::Msg_AE * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;

	messageOut->StatusWord1.StatusWord2ID = messageIn.StatusWord1.StatusWord2ID;
	messageOut->StatusWord1.Control_Frequency.value = static_cast<uint8_t>(messageIn.StatusWord1.Control_Frequency);
	messageOut->StatusWord1.Guidance_Frequency.value = static_cast<uint8_t>(messageIn.StatusWord1.Guidance_Frequency);
	messageOut->StatusWord1.Gyro_BIT_Summary = messageIn.StatusWord1.Gyro_BIT_Summary;
	messageOut->StatusWord1.Accelerometer_BIT_Summary = messageIn.StatusWord1.Accelerometer_BIT_Summary;
	messageOut->StatusWord1.Magnetometer_BIT_Summary = messageIn.StatusWord1.Magnetometer_BIT_Summary;
	messageOut->StatusWord1.CBIT_Status = messageIn.StatusWord1.CBIT_Status;

	messageOut->MultiPlexedStatusWord2.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord2.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord2.DeviceId = messageIn.MultiPlexedStatusWord2.DeviceId;
	messageOut->MultiPlexedStatusWord2.PerformanceGrade = messageIn.MultiPlexedStatusWord2.PerformanceGrade;
	messageOut->MultiPlexedStatusWord2.Gyro_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Gyro_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Gyro_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Gyro_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.Accel_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Accel_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Accel_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Accel_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.Mag_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Mag_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Mag_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Mag_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.NormalModePrimaryCRC = messageIn.MultiPlexedStatusWord2.NormalModePrimaryCRC;
	messageOut->MultiPlexedStatusWord2.NormalModeSecondayrCRC = messageIn.MultiPlexedStatusWord2.NormalModeSecondayrCRC;
	messageOut->MultiPlexedStatusWord2.FactoryConfigCRC = messageIn.MultiPlexedStatusWord2.FactoryConfigCRC;
	messageOut->MultiPlexedStatusWord2.FactoryCoefficientCRC = messageIn.MultiPlexedStatusWord2.FactoryCoefficientCRC;
	messageOut->MultiPlexedStatusWord2.IO_ConfigCRC = messageIn.MultiPlexedStatusWord2.IO_ConfigCRC;
	messageOut->MultiPlexedStatusWord2.PrimaryImageBoot = messageIn.MultiPlexedStatusWord2.PrimaryImageBoot;
	messageOut->MultiPlexedStatusWord2.MemoryTestSummary = messageIn.MultiPlexedStatusWord2.MemoryTestSummary;
	messageOut->MultiPlexedStatusWord2.ProcessorTestSummary = messageIn.MultiPlexedStatusWord2.ProcessorTestSummary;
	messageOut->MultiPlexedStatusWord2.WdtLoopCompletionSummary = messageIn.MultiPlexedStatusWord2.WdtLoopCompletionSummary;
	messageOut->MultiPlexedStatusWord2.PowerUpBitStatus = messageIn.MultiPlexedStatusWord2.PowerUpBitStatus;
	messageOut->MultiPlexedStatusWord2.ContinuousBitStatus = messageIn.MultiPlexedStatusWord2.ContinuousBitStatus;
	messageOut->MultiPlexedStatusWord2.DeviceTemperature = messageIn.MultiPlexedStatusWord2.DeviceTemperature;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_AE
void convert(hg_node::Msg_AE messageIn, Msg_AE * messageOut)
{
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->MagneticFieldX = messageIn.MagneticFieldX;
	messageOut->MagneticFieldY = messageIn.MagneticFieldY;
	messageOut->MagneticFieldZ = messageIn.MagneticFieldZ;

	messageOut->StatusWord1.StatusWord2ID = messageIn.StatusWord1.StatusWord2ID;
	messageOut->StatusWord1.Control_Frequency = static_cast<control_frequency_t>(messageIn.StatusWord1.Control_Frequency.value);
	messageOut->StatusWord1.Guidance_Frequency = static_cast<guidance_frequency_t>(messageIn.StatusWord1.Guidance_Frequency.value);
	messageOut->StatusWord1.Gyro_BIT_Summary = messageIn.StatusWord1.Gyro_BIT_Summary;
	messageOut->StatusWord1.Accelerometer_BIT_Summary = messageIn.StatusWord1.Accelerometer_BIT_Summary;
	messageOut->StatusWord1.Magnetometer_BIT_Summary = messageIn.StatusWord1.Magnetometer_BIT_Summary;
	messageOut->StatusWord1.CBIT_Status = messageIn.StatusWord1.CBIT_Status;

	messageOut->MultiPlexedStatusWord2.EmbeddedSoftwareVersion = messageIn.MultiPlexedStatusWord2.EmbeddedSoftwareVersion;
	messageOut->MultiPlexedStatusWord2.DeviceId = messageIn.MultiPlexedStatusWord2.DeviceId;
	messageOut->MultiPlexedStatusWord2.PerformanceGrade = messageIn.MultiPlexedStatusWord2.PerformanceGrade;
	messageOut->MultiPlexedStatusWord2.Gyro_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Gyro_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Gyro_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Gyro_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.Accel_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Accel_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Accel_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Accel_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.Mag_StatisticsSummary = messageIn.MultiPlexedStatusWord2.Mag_StatisticsSummary;
	messageOut->MultiPlexedStatusWord2.Mag_TemperatureSummary = messageIn.MultiPlexedStatusWord2.Mag_TemperatureSummary;
	messageOut->MultiPlexedStatusWord2.NormalModePrimaryCRC = messageIn.MultiPlexedStatusWord2.NormalModePrimaryCRC;
	messageOut->MultiPlexedStatusWord2.NormalModeSecondayrCRC = messageIn.MultiPlexedStatusWord2.NormalModeSecondayrCRC;
	messageOut->MultiPlexedStatusWord2.FactoryConfigCRC = messageIn.MultiPlexedStatusWord2.FactoryConfigCRC;
	messageOut->MultiPlexedStatusWord2.FactoryCoefficientCRC = messageIn.MultiPlexedStatusWord2.FactoryCoefficientCRC;
	messageOut->MultiPlexedStatusWord2.IO_ConfigCRC = messageIn.MultiPlexedStatusWord2.IO_ConfigCRC;
	messageOut->MultiPlexedStatusWord2.PrimaryImageBoot = messageIn.MultiPlexedStatusWord2.PrimaryImageBoot;
	messageOut->MultiPlexedStatusWord2.MemoryTestSummary = messageIn.MultiPlexedStatusWord2.MemoryTestSummary;
	messageOut->MultiPlexedStatusWord2.ProcessorTestSummary = messageIn.MultiPlexedStatusWord2.ProcessorTestSummary;
	messageOut->MultiPlexedStatusWord2.WdtLoopCompletionSummary = messageIn.MultiPlexedStatusWord2.WdtLoopCompletionSummary;
	messageOut->MultiPlexedStatusWord2.PowerUpBitStatus = messageIn.MultiPlexedStatusWord2.PowerUpBitStatus;
	messageOut->MultiPlexedStatusWord2.ContinuousBitStatus = messageIn.MultiPlexedStatusWord2.ContinuousBitStatus;
	messageOut->MultiPlexedStatusWord2.DeviceTemperature = messageIn.MultiPlexedStatusWord2.DeviceTemperature;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_AE_pub_callback(uint8_t * buffer)
{
	Msg_AE Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xAE deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_AE);
	ROS_DEBUG("Message 0xAE Received");

	// Initialize Publisher if not initialized yet
	if (Msg_AE_pub_initialized == false){
		init_AE(getRosHandle());}
	// Publish the message
	Msg_AE_pub.publish(msgStruct_AE);
	return;
}
