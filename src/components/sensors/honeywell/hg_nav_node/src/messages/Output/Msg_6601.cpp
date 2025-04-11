#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6601.h>
#include <hg_nav_node/led_bitfield_t.h>
hg_nav_node::Msg_6601 msgStruct_6601;

bool Msg_6601_pub_initialized = false;

ros::Publisher Msg_6601_pub;
void init_6601(ros::NodeHandle * n){
	Msg_6601_pub = n->advertise<hg_nav_node::Msg_6601>(MSG_6601_PATH, 5);
	Msg_6601_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6601_PATH);
	return;
}

void stop_6601(void){
	Msg_6601_pub.shutdown();
	Msg_6601_pub_initialized = false;
	ROS_INFO("0x6601 stopped");
	return;
}

// Msg_6601 to Topic
void convert(Msg_6601 messageIn, hg_nav_node::Msg_6601 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->PWR_LED_State.blue_byte = messageIn.PWR_LED_State.blue_byte;
	messageOut->PWR_LED_State.green_byte = messageIn.PWR_LED_State.green_byte;
	messageOut->PWR_LED_State.red_byte = messageIn.PWR_LED_State.red_byte;
	messageOut->PWR_LED_State.blue_byte = messageIn.PWR_LED_State.blue_byte;
	messageOut->PWR_LED_State.green_byte = messageIn.PWR_LED_State.green_byte;
	messageOut->PWR_LED_State.red_byte = messageIn.PWR_LED_State.red_byte;

	messageOut->POS_LED_State.blue_byte = messageIn.POS_LED_State.blue_byte;
	messageOut->POS_LED_State.green_byte = messageIn.POS_LED_State.green_byte;
	messageOut->POS_LED_State.red_byte = messageIn.POS_LED_State.red_byte;
	messageOut->POS_LED_State.blue_byte = messageIn.POS_LED_State.blue_byte;
	messageOut->POS_LED_State.green_byte = messageIn.POS_LED_State.green_byte;
	messageOut->POS_LED_State.red_byte = messageIn.POS_LED_State.red_byte;

	messageOut->SYS_LED_State.blue_byte = messageIn.SYS_LED_State.blue_byte;
	messageOut->SYS_LED_State.green_byte = messageIn.SYS_LED_State.green_byte;
	messageOut->SYS_LED_State.red_byte = messageIn.SYS_LED_State.red_byte;
	messageOut->SYS_LED_State.blue_byte = messageIn.SYS_LED_State.blue_byte;
	messageOut->SYS_LED_State.green_byte = messageIn.SYS_LED_State.green_byte;
	messageOut->SYS_LED_State.red_byte = messageIn.SYS_LED_State.red_byte;

	messageOut->LOG_LED_State.blue_byte = messageIn.LOG_LED_State.blue_byte;
	messageOut->LOG_LED_State.green_byte = messageIn.LOG_LED_State.green_byte;
	messageOut->LOG_LED_State.red_byte = messageIn.LOG_LED_State.red_byte;
	messageOut->LOG_LED_State.blue_byte = messageIn.LOG_LED_State.blue_byte;
	messageOut->LOG_LED_State.green_byte = messageIn.LOG_LED_State.green_byte;
	messageOut->LOG_LED_State.red_byte = messageIn.LOG_LED_State.red_byte;
}

// Topic to Msg_6601
void convert(hg_nav_node::Msg_6601 messageIn, Msg_6601 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->PWR_LED_State.blue_byte = messageIn.PWR_LED_State.blue_byte;
	messageOut->PWR_LED_State.green_byte = messageIn.PWR_LED_State.green_byte;
	messageOut->PWR_LED_State.red_byte = messageIn.PWR_LED_State.red_byte;
	messageOut->PWR_LED_State.blue_byte = messageIn.PWR_LED_State.blue_byte;
	messageOut->PWR_LED_State.green_byte = messageIn.PWR_LED_State.green_byte;
	messageOut->PWR_LED_State.red_byte = messageIn.PWR_LED_State.red_byte;

	messageOut->POS_LED_State.blue_byte = messageIn.POS_LED_State.blue_byte;
	messageOut->POS_LED_State.green_byte = messageIn.POS_LED_State.green_byte;
	messageOut->POS_LED_State.red_byte = messageIn.POS_LED_State.red_byte;
	messageOut->POS_LED_State.blue_byte = messageIn.POS_LED_State.blue_byte;
	messageOut->POS_LED_State.green_byte = messageIn.POS_LED_State.green_byte;
	messageOut->POS_LED_State.red_byte = messageIn.POS_LED_State.red_byte;

	messageOut->SYS_LED_State.blue_byte = messageIn.SYS_LED_State.blue_byte;
	messageOut->SYS_LED_State.green_byte = messageIn.SYS_LED_State.green_byte;
	messageOut->SYS_LED_State.red_byte = messageIn.SYS_LED_State.red_byte;
	messageOut->SYS_LED_State.blue_byte = messageIn.SYS_LED_State.blue_byte;
	messageOut->SYS_LED_State.green_byte = messageIn.SYS_LED_State.green_byte;
	messageOut->SYS_LED_State.red_byte = messageIn.SYS_LED_State.red_byte;

	messageOut->LOG_LED_State.blue_byte = messageIn.LOG_LED_State.blue_byte;
	messageOut->LOG_LED_State.green_byte = messageIn.LOG_LED_State.green_byte;
	messageOut->LOG_LED_State.red_byte = messageIn.LOG_LED_State.red_byte;
	messageOut->LOG_LED_State.blue_byte = messageIn.LOG_LED_State.blue_byte;
	messageOut->LOG_LED_State.green_byte = messageIn.LOG_LED_State.green_byte;
	messageOut->LOG_LED_State.red_byte = messageIn.LOG_LED_State.red_byte;
}

void Msg_6601_pub_callback(uint8_t * buffer)
{
	Msg_6601 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6601 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6601);
	ROS_DEBUG("Message 0x6601 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6601_pub_initialized == false){
		init_6601(getRosHandle());}
	// Publish the message
	Msg_6601_pub.publish(msgStruct_6601);
	return;
}
