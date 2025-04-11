#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6001.h>
hg_nav_node::Msg_6001 msgStruct_6001;

bool Msg_6001_pub_initialized = false;

ros::Publisher Msg_6001_pub;
void init_6001(ros::NodeHandle * n){
	Msg_6001_pub = n->advertise<hg_nav_node::Msg_6001>(MSG_6001_PATH, 5);
	Msg_6001_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6001_PATH);
	return;
}

void stop_6001(void){
	Msg_6001_pub.shutdown();
	Msg_6001_pub_initialized = false;
	ROS_INFO("0x6001 stopped");
	return;
}

// Msg_6001 to Topic
void convert(Msg_6001 messageIn, hg_nav_node::Msg_6001 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->DeviceSerialNumber[index] = messageIn.DeviceSerialNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->DevicePartNumber[index] = messageIn.DevicePartNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SensorAssyPartNumber[index] = messageIn.SensorAssyPartNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->NavSoftwareVersion[index] = messageIn.NavSoftwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->NavSoftwareBuildDate[index] = messageIn.NavSoftwareBuildDate[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSerialNumber[index] = messageIn.IMUSerialNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUPartNumber[index] = messageIn.IMUPartNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSoftwareVersion[index] = messageIn.IMUSoftwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverSerialNumber[index] = messageIn.GNSSReceiverSerialNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverPartNumber[index] = messageIn.GNSSReceiverPartNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverFirmwareVersion[index] = messageIn.GNSSReceiverFirmwareVersion[index];
	}

	for (unsigned int index = 0; index < 32; index++)
	{
		messageOut->ProcessorHWIdentifier[index] = messageIn.ProcessorHWIdentifier[index];
	}

	for (unsigned int index = 0; index < 32; index++)
	{
		messageOut->InterconnectHWIdentifier[index] = messageIn.InterconnectHWIdentifier[index];
	}

}

// Topic to Msg_6001
void convert(hg_nav_node::Msg_6001 messageIn, Msg_6001 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->DeviceSerialNumber[index] = messageIn.DeviceSerialNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->DevicePartNumber[index] = messageIn.DevicePartNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SensorAssyPartNumber[index] = messageIn.SensorAssyPartNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->NavSoftwareVersion[index] = messageIn.NavSoftwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->NavSoftwareBuildDate[index] = messageIn.NavSoftwareBuildDate[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSerialNumber[index] = messageIn.IMUSerialNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUPartNumber[index] = messageIn.IMUPartNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSoftwareVersion[index] = messageIn.IMUSoftwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverSerialNumber[index] = messageIn.GNSSReceiverSerialNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverPartNumber[index] = messageIn.GNSSReceiverPartNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSReceiverFirmwareVersion[index] = messageIn.GNSSReceiverFirmwareVersion[index];
	}
	for (unsigned int index = 0; index < 32; index++)
	{
		messageOut->ProcessorHWIdentifier[index] = messageIn.ProcessorHWIdentifier[index];
	}
	for (unsigned int index = 0; index < 32; index++)
	{
		messageOut->InterconnectHWIdentifier[index] = messageIn.InterconnectHWIdentifier[index];
	}

}

void Msg_6001_pub_callback(uint8_t * buffer)
{
	Msg_6001 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6001 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6001);
	ROS_DEBUG("Message 0x6001 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6001_pub_initialized == false){
		init_6001(getRosHandle());}
	// Publish the message
	Msg_6001_pub.publish(msgStruct_6001);
	return;
}
