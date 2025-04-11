#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5103.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5103 msgStruct_5103;

bool Msg_5103_pub_initialized = false;

ros::Publisher Msg_5103_pub;
void init_5103(ros::NodeHandle * n){
	Msg_5103_pub = n->advertise<hg_nav_node::Msg_5103>(MSG_5103_PATH, 5);
	Msg_5103_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5103_PATH);
	return;
}

void stop_5103(void){
	Msg_5103_pub.shutdown();
	Msg_5103_pub_initialized = false;
	ROS_INFO("0x5103 stopped");
	return;
}

// Msg_5103 to Topic
void convert(Msg_5103 messageIn, hg_nav_node::Msg_5103 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->prn_number = messageIn.prn_number;
	messageOut->tow = messageIn.tow;
	messageOut->health = messageIn.health;
	messageOut->IODE1 = messageIn.IODE1;
	messageOut->IODE2 = messageIn.IODE2;
	messageOut->week = messageIn.week;
	messageOut->z_week = messageIn.z_week;
	messageOut->toe = messageIn.toe;
	messageOut->A = messageIn.A;
	messageOut->dN = messageIn.dN;
	messageOut->M0 = messageIn.M0;
	messageOut->ecc = messageIn.ecc;
	messageOut->w = messageIn.w;
	messageOut->cuc = messageIn.cuc;
	messageOut->cus = messageIn.cus;
	messageOut->crc = messageIn.crc;
	messageOut->crs = messageIn.crs;
	messageOut->cic = messageIn.cic;
	messageOut->cis = messageIn.cis;
	messageOut->Inclination_angle = messageIn.Inclination_angle;
	messageOut->Inclination_rate = messageIn.Inclination_rate;
	messageOut->wo = messageIn.wo;
	messageOut->wi = messageIn.wi;
	messageOut->iodc = messageIn.iodc;
	messageOut->toc = messageIn.toc;
	messageOut->tgd = messageIn.tgd;
	messageOut->af0 = messageIn.af0;
	messageOut->af1 = messageIn.af1;
	messageOut->af2 = messageIn.af2;
	messageOut->AS = messageIn.AS;
	messageOut->N = messageIn.N;
	messageOut->URA = messageIn.URA;
}

// Topic to Msg_5103
void convert(hg_nav_node::Msg_5103 messageIn, Msg_5103 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->prn_number = messageIn.prn_number;
	messageOut->tow = messageIn.tow;
	messageOut->health = messageIn.health;
	messageOut->IODE1 = messageIn.IODE1;
	messageOut->IODE2 = messageIn.IODE2;
	messageOut->week = messageIn.week;
	messageOut->z_week = messageIn.z_week;
	messageOut->toe = messageIn.toe;
	messageOut->A = messageIn.A;
	messageOut->dN = messageIn.dN;
	messageOut->M0 = messageIn.M0;
	messageOut->ecc = messageIn.ecc;
	messageOut->w = messageIn.w;
	messageOut->cuc = messageIn.cuc;
	messageOut->cus = messageIn.cus;
	messageOut->crc = messageIn.crc;
	messageOut->crs = messageIn.crs;
	messageOut->cic = messageIn.cic;
	messageOut->cis = messageIn.cis;
	messageOut->Inclination_angle = messageIn.Inclination_angle;
	messageOut->Inclination_rate = messageIn.Inclination_rate;
	messageOut->wo = messageIn.wo;
	messageOut->wi = messageIn.wi;
	messageOut->iodc = messageIn.iodc;
	messageOut->toc = messageIn.toc;
	messageOut->tgd = messageIn.tgd;
	messageOut->af0 = messageIn.af0;
	messageOut->af1 = messageIn.af1;
	messageOut->af2 = messageIn.af2;
	messageOut->AS = messageIn.AS;
	messageOut->N = messageIn.N;
	messageOut->URA = messageIn.URA;
}

void Msg_5103_pub_callback(uint8_t * buffer)
{
	Msg_5103 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5103 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5103);
	ROS_DEBUG("Message 0x5103 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5103_pub_initialized == false){
		init_5103(getRosHandle());}
	// Publish the message
	Msg_5103_pub.publish(msgStruct_5103);
	return;
}
