#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5012.h>
hg_nav_node::Msg_5012 msgStruct_5012;

bool Msg_5012_pub_initialized = false;

ros::Publisher Msg_5012_pub;
void init_5012(ros::NodeHandle * n){
	Msg_5012_pub = n->advertise<hg_nav_node::Msg_5012>(MSG_5012_PATH, 5);
	Msg_5012_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5012_PATH);
	return;
}

void stop_5012(void){
	Msg_5012_pub.shutdown();
	Msg_5012_pub_initialized = false;
	ROS_INFO("0x5012 stopped");
	return;
}

// Msg_5012 to Topic
void convert(Msg_5012 messageIn, hg_nav_node::Msg_5012 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->Temperature = messageIn.Temperature;
	messageOut->RF1_SupplyVoltage = messageIn.RF1_SupplyVoltage;
	messageOut->RF1_SupplyCurrent = messageIn.RF1_SupplyCurrent;
	messageOut->RF2_SupplyVoltage = messageIn.RF2_SupplyVoltage;
	messageOut->RF2_SupplyCurrent = messageIn.RF2_SupplyCurrent;
	messageOut->Voltage3V3 = messageIn.Voltage3V3;
	messageOut->Voltage1V8 = messageIn.Voltage1V8;
	messageOut->Voltage1V2 = messageIn.Voltage1V2;
	messageOut->Voltage5V0 = messageIn.Voltage5V0;
	messageOut->ptc = messageIn.ptc;
	messageOut->ptm = messageIn.ptm;
	messageOut->memUsage = messageIn.memUsage;
	messageOut->mode = messageIn.mode;
	messageOut->power_cycle_count = messageIn.power_cycle_count;
	messageOut->eti = messageIn.eti;
}

// Topic to Msg_5012
void convert(hg_nav_node::Msg_5012 messageIn, Msg_5012 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->Temperature = messageIn.Temperature;
	messageOut->RF1_SupplyVoltage = messageIn.RF1_SupplyVoltage;
	messageOut->RF1_SupplyCurrent = messageIn.RF1_SupplyCurrent;
	messageOut->RF2_SupplyVoltage = messageIn.RF2_SupplyVoltage;
	messageOut->RF2_SupplyCurrent = messageIn.RF2_SupplyCurrent;
	messageOut->Voltage3V3 = messageIn.Voltage3V3;
	messageOut->Voltage1V8 = messageIn.Voltage1V8;
	messageOut->Voltage1V2 = messageIn.Voltage1V2;
	messageOut->Voltage5V0 = messageIn.Voltage5V0;
	messageOut->ptc = messageIn.ptc;
	messageOut->ptm = messageIn.ptm;
	messageOut->memUsage = messageIn.memUsage;
	messageOut->mode = messageIn.mode;
	messageOut->power_cycle_count = messageIn.power_cycle_count;
	messageOut->eti = messageIn.eti;
}

void Msg_5012_pub_callback(uint8_t * buffer)
{
	Msg_5012 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5012 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5012);
	ROS_DEBUG("Message 0x5012 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5012_pub_initialized == false){
		init_5012(getRosHandle());}
	// Publish the message
	Msg_5012_pub.publish(msgStruct_5012);
	return;
}
