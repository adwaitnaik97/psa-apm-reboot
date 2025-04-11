#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5001.h>
hg_nav_node::Msg_5001 msgStruct_5001;

bool Msg_5001_pub_initialized = false;

ros::Publisher Msg_5001_pub;
void init_5001(ros::NodeHandle * n){
	Msg_5001_pub = n->advertise<hg_nav_node::Msg_5001>(MSG_5001_PATH, 5);
	Msg_5001_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5001_PATH);
	return;
}

void stop_5001(void){
	Msg_5001_pub.shutdown();
	Msg_5001_pub_initialized = false;
	ROS_INFO("0x5001 stopped");
	return;
}

// Msg_5001 to Topic
void convert(Msg_5001 messageIn, hg_nav_node::Msg_5001 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SerialNumber[index] = messageIn.SerialNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->PartNumber[index] = messageIn.PartNumber[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Model[index] = messageIn.Model[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSHardwareVersion[index] = messageIn.GNSSHardwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareVersion[index] = messageIn.FirmwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareBuildDate[index] = messageIn.FirmwareBuildDate[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareBuildTime[index] = messageIn.FirmwareBuildTime[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component1Type[index] = messageIn.Component1Type[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component1Version[index] = messageIn.Component1Version[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component2Type[index] = messageIn.Component2Type[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component2Version[index] = messageIn.Component2Version[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component3Type[index] = messageIn.Component3Type[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component3Version[index] = messageIn.Component3Version[index];
	}

}

// Topic to Msg_5001
void convert(hg_nav_node::Msg_5001 messageIn, Msg_5001 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SerialNumber[index] = messageIn.SerialNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->PartNumber[index] = messageIn.PartNumber[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Model[index] = messageIn.Model[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->GNSSHardwareVersion[index] = messageIn.GNSSHardwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareVersion[index] = messageIn.FirmwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareBuildDate[index] = messageIn.FirmwareBuildDate[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FirmwareBuildTime[index] = messageIn.FirmwareBuildTime[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component1Type[index] = messageIn.Component1Type[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component1Version[index] = messageIn.Component1Version[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component2Type[index] = messageIn.Component2Type[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component2Version[index] = messageIn.Component2Version[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component3Type[index] = messageIn.Component3Type[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->Component3Version[index] = messageIn.Component3Version[index];
	}

}

void Msg_5001_pub_callback(uint8_t * buffer)
{
	Msg_5001 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5001 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5001);
	ROS_DEBUG("Message 0x5001 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5001_pub_initialized == false){
		init_5001(getRosHandle());}
	// Publish the message
	Msg_5001_pub.publish(msgStruct_5001);
	return;
}
