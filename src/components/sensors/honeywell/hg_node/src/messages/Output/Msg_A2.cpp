#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_node/Msg_A2.h>
#include <hg_node/control_frequency_t.h>
#include <hg_node/guidance_frequency_t.h>
#include <hg_node/hgimu_multiplex_status_word2_t.h>
#include <hg_node/hgimu_status_word_1_t.h>
hg_node::Msg_A2 msgStruct_A2;

bool Msg_A2_pub_initialized = false;

ros::Publisher Msg_A2_pub;
void init_A2(ros::NodeHandle * n){
	Msg_A2_pub = n->advertise<hg_node::Msg_A2>(MSG_A2_PATH, 5);
	Msg_A2_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_A2_PATH);
	return;
}

void stop_A2(void){
	if (Msg_A2_pub_initialized){
		Msg_A2_pub.shutdown();
		Msg_A2_pub_initialized = false;
		ROS_INFO("0xA2 stopped");
	}
	return;
}

// Msg_A2 to Topic
void convert(Msg_A2 messageIn, hg_node::Msg_A2 * messageOut)
{
	messageOut->SyncByte = messageIn.SyncByte;
	messageOut->MessageID = messageIn.MessageID;
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;

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
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->Checksum = messageIn.Checksum;
}

// Topic to Msg_A2
void convert(hg_node::Msg_A2 messageIn, Msg_A2 * messageOut)
{
	messageOut->AngularRateX = messageIn.AngularRateX;
	messageOut->AngularRateY = messageIn.AngularRateY;
	messageOut->AngularRateZ = messageIn.AngularRateZ;
	messageOut->LinearAccelerationX = messageIn.LinearAccelerationX;
	messageOut->LinearAccelerationY = messageIn.LinearAccelerationY;
	messageOut->LinearAccelerationZ = messageIn.LinearAccelerationZ;

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
	messageOut->DeltaAngleX = messageIn.DeltaAngleX;
	messageOut->DeltaAngleY = messageIn.DeltaAngleY;
	messageOut->DeltaAngleZ = messageIn.DeltaAngleZ;
	messageOut->DeltaVelocityX = messageIn.DeltaVelocityX;
	messageOut->DeltaVelocityY = messageIn.DeltaVelocityY;
	messageOut->DeltaVelocityZ = messageIn.DeltaVelocityZ;
	messageOut->Checksum = messageIn.Checksum;
}

void Msg_A2_pub_callback(uint8_t * buffer)
{
	Msg_A2 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0xA2 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_A2);
	ROS_DEBUG("Message 0xA2 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_A2_pub_initialized == false){
		init_A2(getRosHandle());}
	// Publish the message
	Msg_A2_pub.publish(msgStruct_A2);
	return;
}
