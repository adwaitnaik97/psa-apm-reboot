#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5104.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5104 msgStruct_5104;

bool Msg_5104_pub_initialized = false;

ros::Publisher Msg_5104_pub;
void init_5104(ros::NodeHandle * n){
	Msg_5104_pub = n->advertise<hg_nav_node::Msg_5104>(MSG_5104_PATH, 5);
	Msg_5104_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5104_PATH);
	return;
}

void stop_5104(void){
	Msg_5104_pub.shutdown();
	Msg_5104_pub_initialized = false;
	ROS_INFO("0x5104 stopped");
	return;
}

// Msg_5104 to Topic
void convert(Msg_5104 messageIn, hg_nav_node::Msg_5104 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->slot_offset = messageIn.slot_offset;
	messageOut->freq_offset = messageIn.freq_offset;
	messageOut->sat_type = messageIn.sat_type;
	messageOut->e_week = messageIn.e_week;
	messageOut->e_time = messageIn.e_time;
	messageOut->t_offset = messageIn.t_offset;
	messageOut->Nt = messageIn.Nt;
	messageOut->issue = messageIn.issue;
	messageOut->health1 = messageIn.health1;
	messageOut->pos_x = messageIn.pos_x;
	messageOut->pos_y = messageIn.pos_y;
	messageOut->pos_z = messageIn.pos_z;
	messageOut->vel_x = messageIn.vel_x;
	messageOut->vel_y = messageIn.vel_y;
	messageOut->vel_z = messageIn.vel_z;
	messageOut->LS_acc_x = messageIn.LS_acc_x;
	messageOut->LS_acc_y = messageIn.LS_acc_y;
	messageOut->LS_acc_z = messageIn.LS_acc_z;
	messageOut->tau_n = messageIn.tau_n;
	messageOut->delta_tau_n = messageIn.delta_tau_n;
	messageOut->gamma = messageIn.gamma;
	messageOut->Tk = messageIn.Tk;
	messageOut->P = messageIn.P;
	messageOut->Ft = messageIn.Ft;
	messageOut->age = messageIn.age;
	messageOut->Flags = messageIn.Flags;
	messageOut->flag_p1 = messageIn.flag_p1;
	messageOut->flag_p2 = messageIn.flag_p2;
	messageOut->flag_p3 = messageIn.flag_p3;
	messageOut->flag_p4 = messageIn.flag_p4;
}

// Topic to Msg_5104
void convert(hg_nav_node::Msg_5104 messageIn, Msg_5104 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->slot_offset = messageIn.slot_offset;
	messageOut->freq_offset = messageIn.freq_offset;
	messageOut->sat_type = messageIn.sat_type;
	messageOut->e_week = messageIn.e_week;
	messageOut->e_time = messageIn.e_time;
	messageOut->t_offset = messageIn.t_offset;
	messageOut->Nt = messageIn.Nt;
	messageOut->issue = messageIn.issue;
	messageOut->health1 = messageIn.health1;
	messageOut->pos_x = messageIn.pos_x;
	messageOut->pos_y = messageIn.pos_y;
	messageOut->pos_z = messageIn.pos_z;
	messageOut->vel_x = messageIn.vel_x;
	messageOut->vel_y = messageIn.vel_y;
	messageOut->vel_z = messageIn.vel_z;
	messageOut->LS_acc_x = messageIn.LS_acc_x;
	messageOut->LS_acc_y = messageIn.LS_acc_y;
	messageOut->LS_acc_z = messageIn.LS_acc_z;
	messageOut->tau_n = messageIn.tau_n;
	messageOut->delta_tau_n = messageIn.delta_tau_n;
	messageOut->gamma = messageIn.gamma;
	messageOut->Tk = messageIn.Tk;
	messageOut->P = messageIn.P;
	messageOut->Ft = messageIn.Ft;
	messageOut->age = messageIn.age;
	messageOut->Flags = messageIn.Flags;
	messageOut->flag_p1 = messageIn.flag_p1;
	messageOut->flag_p2 = messageIn.flag_p2;
	messageOut->flag_p3 = messageIn.flag_p3;
	messageOut->flag_p4 = messageIn.flag_p4;
}

void Msg_5104_pub_callback(uint8_t * buffer)
{
	Msg_5104 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5104 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5104);
	ROS_DEBUG("Message 0x5104 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5104_pub_initialized == false){
		init_5104(getRosHandle());}
	// Publish the message
	Msg_5104_pub.publish(msgStruct_5104);
	return;
}
