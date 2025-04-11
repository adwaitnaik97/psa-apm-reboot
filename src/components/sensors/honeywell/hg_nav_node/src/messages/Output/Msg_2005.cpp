#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2005.h>
#include <hg_nav_node/interface_protocol_t.h>
#include <hg_nav_node/port_id_t.h>
#include <hg_nav_node/save_configuration_t.h>
#include <hg_nav_node/uart_baud_type_t.h>
#include <hg_nav_node/uart_number_of_bits_t.h>
#include <hg_nav_node/uart_number_of_stop_bits_t.h>
#include <hg_nav_node/uart_parity_t.h>
hg_nav_node::Msg_2005 msgStruct_2005;

bool Msg_2005_pub_initialized = false;

ros::Publisher Msg_2005_pub;
void init_2005(ros::NodeHandle * n){
	Msg_2005_pub = n->advertise<hg_nav_node::Msg_2005>(MSG_2005_PATH, 5);
	Msg_2005_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2005_PATH);
	return;
}

void stop_2005(void){
	Msg_2005_pub.shutdown();
	Msg_2005_pub_initialized = false;
	ROS_INFO("0x2005 stopped");
	return;
}

// Msg_2005 to Topic
void convert(Msg_2005 messageIn, hg_nav_node::Msg_2005 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->PortId.value = static_cast<uint8_t>(messageIn.PortId);
	messageOut->InterfaceSelect.value = static_cast<uint8_t>(messageIn.InterfaceSelect);
	messageOut->BaudRate.value = static_cast<uint8_t>(messageIn.BaudRate);
	messageOut->NumberOfBits.value = static_cast<uint8_t>(messageIn.NumberOfBits);
	messageOut->Parity.value = static_cast<uint8_t>(messageIn.Parity);
	messageOut->NumberOfStopBits.value = static_cast<uint8_t>(messageIn.NumberOfStopBits);
	messageOut->SaveConfiguration.value = static_cast<uint8_t>(messageIn.SaveConfiguration);
	messageOut->parameter_1 = messageIn.parameter_1;
	messageOut->parameter_2 = messageIn.parameter_2;
	messageOut->parameter_3 = messageIn.parameter_3;
	messageOut->parameter_4 = messageIn.parameter_4;
}

// Topic to Msg_2005
void convert(hg_nav_node::Msg_2005 messageIn, Msg_2005 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->PortId = static_cast<port_id_t>(messageIn.PortId.value);
	messageOut->InterfaceSelect = static_cast<interface_protocol_t>(messageIn.InterfaceSelect.value);
	messageOut->BaudRate = static_cast<uart_baud_type_t>(messageIn.BaudRate.value);
	messageOut->NumberOfBits = static_cast<uart_number_of_bits_t>(messageIn.NumberOfBits.value);
	messageOut->Parity = static_cast<uart_parity_t>(messageIn.Parity.value);
	messageOut->NumberOfStopBits = static_cast<uart_number_of_stop_bits_t>(messageIn.NumberOfStopBits.value);
	messageOut->SaveConfiguration = static_cast<save_configuration_t>(messageIn.SaveConfiguration.value);
	messageOut->parameter_1 = messageIn.parameter_1;
	messageOut->parameter_2 = messageIn.parameter_2;
	messageOut->parameter_3 = messageIn.parameter_3;
	messageOut->parameter_4 = messageIn.parameter_4;
}

void Msg_2005_pub_callback(uint8_t * buffer)
{
	Msg_2005 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2005 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2005);
	ROS_DEBUG("Message 0x2005 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2005_pub_initialized == false){
		init_2005(getRosHandle());}
	// Publish the message
	Msg_2005_pub.publish(msgStruct_2005);
	return;
}
