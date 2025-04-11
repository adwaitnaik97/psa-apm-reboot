#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"
#include <vector> //Repeat Var

// Include custom message
#include <hg_nav_node/Msg_5102.h>
#include <hg_nav_node/constellation_enum_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/satellite_pos_t.h>
hg_nav_node::Msg_5102 msgStruct_5102;

bool Msg_5102_pub_initialized = false;

ros::Publisher Msg_5102_pub;
void init_5102(ros::NodeHandle * n){
	Msg_5102_pub = n->advertise<hg_nav_node::Msg_5102>(MSG_5102_PATH, 5);
	Msg_5102_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5102_PATH);
	return;
}

void stop_5102(void){
	Msg_5102_pub.shutdown();
	Msg_5102_pub_initialized = false;
	ROS_INFO("0x5102 stopped");
	return;
}

// Msg_5102 to Topic
void convert(Msg_5102 messageIn, hg_nav_node::Msg_5102 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->num_satellites = messageIn.num_satellites;

	//Variable lenght field
	messageOut->sat_position.resize(messageIn.get_sat_position_size());
	for (unsigned int i = 0; i < messageIn.get_sat_position_size(); i++){

	messageOut->sat_position[i].system.value = static_cast<uint8_t>(messageIn.sat_position[i].system);
	messageOut->sat_position[i].satelliteID = messageIn.sat_position[i].satelliteID;
	messageOut->sat_position[i].pos_x = messageIn.sat_position[i].pos_x;
	messageOut->sat_position[i].pos_y = messageIn.sat_position[i].pos_y;
	messageOut->sat_position[i].pos_z = messageIn.sat_position[i].pos_z;
	messageOut->sat_position[i].clk_corr = messageIn.sat_position[i].clk_corr;
	messageOut->sat_position[i].iono_delay = messageIn.sat_position[i].iono_delay;
	messageOut->sat_position[i].tropo_delay = messageIn.sat_position[i].tropo_delay;
	messageOut->sat_position[i].system.value = static_cast<uint8_t>(messageIn.sat_position[i].system);
	messageOut->sat_position[i].satelliteID = messageIn.sat_position[i].satelliteID;
	messageOut->sat_position[i].pos_x = messageIn.sat_position[i].pos_x;
	messageOut->sat_position[i].pos_y = messageIn.sat_position[i].pos_y;
	messageOut->sat_position[i].pos_z = messageIn.sat_position[i].pos_z;
	messageOut->sat_position[i].clk_corr = messageIn.sat_position[i].clk_corr;
	messageOut->sat_position[i].iono_delay = messageIn.sat_position[i].iono_delay;
	messageOut->sat_position[i].tropo_delay = messageIn.sat_position[i].tropo_delay;
	}
}

// Topic to Msg_5102
void convert(hg_nav_node::Msg_5102 messageIn, Msg_5102 * messageOut)
{
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->num_satellites = messageIn.num_satellites;

	//Variable lenght field
	uint32_t sat_position_size = (messageIn.sat_position.size()<=messageOut->get_sat_position_size())?messageIn.sat_position.size():messageOut->get_sat_position_size();
	for (unsigned int i = 0; i < sat_position_size; i++){

	messageOut->sat_position[i].system = static_cast<constellation_enum_t>(messageIn.sat_position[i].system.value);
	messageOut->sat_position[i].satelliteID = messageIn.sat_position[i].satelliteID;
	messageOut->sat_position[i].pos_x = messageIn.sat_position[i].pos_x;
	messageOut->sat_position[i].pos_y = messageIn.sat_position[i].pos_y;
	messageOut->sat_position[i].pos_z = messageIn.sat_position[i].pos_z;
	messageOut->sat_position[i].clk_corr = messageIn.sat_position[i].clk_corr;
	messageOut->sat_position[i].iono_delay = messageIn.sat_position[i].iono_delay;
	messageOut->sat_position[i].tropo_delay = messageIn.sat_position[i].tropo_delay;
	messageOut->sat_position[i].system = static_cast<constellation_enum_t>(messageIn.sat_position[i].system.value);
	messageOut->sat_position[i].satelliteID = messageIn.sat_position[i].satelliteID;
	messageOut->sat_position[i].pos_x = messageIn.sat_position[i].pos_x;
	messageOut->sat_position[i].pos_y = messageIn.sat_position[i].pos_y;
	messageOut->sat_position[i].pos_z = messageIn.sat_position[i].pos_z;
	messageOut->sat_position[i].clk_corr = messageIn.sat_position[i].clk_corr;
	messageOut->sat_position[i].iono_delay = messageIn.sat_position[i].iono_delay;
	messageOut->sat_position[i].tropo_delay = messageIn.sat_position[i].tropo_delay;
	}
}

void Msg_5102_pub_callback(uint8_t * buffer)
{
	Msg_5102 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5102 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5102);
	ROS_DEBUG("Message 0x5102 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5102_pub_initialized == false){
		init_5102(getRosHandle());}
	// Publish the message
	Msg_5102_pub.publish(msgStruct_5102);
	return;
}
