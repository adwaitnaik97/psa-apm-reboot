#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5106.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5106 msgStruct_5106;

bool Msg_5106_pub_initialized = false;

ros::Publisher Msg_5106_pub;
void init_5106(ros::NodeHandle * n){
	Msg_5106_pub = n->advertise<hg_nav_node::Msg_5106>(MSG_5106_PATH, 5);
	Msg_5106_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5106_PATH);
	return;
}

void stop_5106(void){
	Msg_5106_pub.shutdown();
	Msg_5106_pub_initialized = false;
	ROS_INFO("0x5106 stopped");
	return;
}

// Msg_5106 to Topic
void convert(Msg_5106 messageIn, hg_nav_node::Msg_5106 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->ephem_type = messageIn.ephem_type;
	messageOut->sat_type = messageIn.sat_type;
	messageOut->satellite_ID = messageIn.satellite_ID;
	messageOut->Week = messageIn.Week;
	messageOut->URA = messageIn.URA;
	messageOut->health1 = messageIn.health1;
	messageOut->tgd1 = messageIn.tgd1;
	messageOut->tgdb2cp = messageIn.tgdb2cp;
	messageOut->tgd2 = messageIn.tgd2;
	messageOut->tgdb2bi = messageIn.tgdb2bi;
	messageOut->AODC = messageIn.AODC;
	messageOut->IODC = messageIn.IODC;
	messageOut->toc = messageIn.toc;
	messageOut->a0 = messageIn.a0;
	messageOut->a1 = messageIn.a1;
	messageOut->a2 = messageIn.a2;
	messageOut->AODE = messageIn.AODE;
	messageOut->toe = messageIn.toe;
	messageOut->RootA = messageIn.RootA;
	messageOut->ecc = messageIn.ecc;
	messageOut->w = messageIn.w;
	messageOut->dN = messageIn.dN;
	messageOut->M0 = messageIn.M0;
	messageOut->omega0 = messageIn.omega0;
	messageOut->omegai = messageIn.omegai;
	messageOut->i0 = messageIn.i0;
	messageOut->IDOT = messageIn.IDOT;
	messageOut->cuc = messageIn.cuc;
	messageOut->cus = messageIn.cus;
	messageOut->crc = messageIn.crc;
	messageOut->crs = messageIn.crs;
	messageOut->cic = messageIn.cic;
	messageOut->cis = messageIn.cis;
	messageOut->tgd3 = messageIn.tgd3;
	messageOut->IscB1cd = messageIn.IscB1cd;
	messageOut->IscB2ad = messageIn.IscB2ad;
	messageOut->SISMAI = messageIn.SISMAI;
	messageOut->deltaA = messageIn.deltaA;
	messageOut->Adot = messageIn.Adot;
	messageOut->deltaN0Dot = messageIn.deltaN0Dot;
}

// Topic to Msg_5106
void convert(hg_nav_node::Msg_5106 messageIn, Msg_5106 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->ephem_type = messageIn.ephem_type;
	messageOut->sat_type = messageIn.sat_type;
	messageOut->satellite_ID = messageIn.satellite_ID;
	messageOut->Week = messageIn.Week;
	messageOut->URA = messageIn.URA;
	messageOut->health1 = messageIn.health1;
	messageOut->tgd1 = messageIn.tgd1;
	messageOut->tgdb2cp = messageIn.tgdb2cp;
	messageOut->tgd2 = messageIn.tgd2;
	messageOut->tgdb2bi = messageIn.tgdb2bi;
	messageOut->AODC = messageIn.AODC;
	messageOut->IODC = messageIn.IODC;
	messageOut->toc = messageIn.toc;
	messageOut->a0 = messageIn.a0;
	messageOut->a1 = messageIn.a1;
	messageOut->a2 = messageIn.a2;
	messageOut->AODE = messageIn.AODE;
	messageOut->toe = messageIn.toe;
	messageOut->RootA = messageIn.RootA;
	messageOut->ecc = messageIn.ecc;
	messageOut->w = messageIn.w;
	messageOut->dN = messageIn.dN;
	messageOut->M0 = messageIn.M0;
	messageOut->omega0 = messageIn.omega0;
	messageOut->omegai = messageIn.omegai;
	messageOut->i0 = messageIn.i0;
	messageOut->IDOT = messageIn.IDOT;
	messageOut->cuc = messageIn.cuc;
	messageOut->cus = messageIn.cus;
	messageOut->crc = messageIn.crc;
	messageOut->crs = messageIn.crs;
	messageOut->cic = messageIn.cic;
	messageOut->cis = messageIn.cis;
	messageOut->tgd3 = messageIn.tgd3;
	messageOut->IscB1cd = messageIn.IscB1cd;
	messageOut->IscB2ad = messageIn.IscB2ad;
	messageOut->SISMAI = messageIn.SISMAI;
	messageOut->deltaA = messageIn.deltaA;
	messageOut->Adot = messageIn.Adot;
	messageOut->deltaN0Dot = messageIn.deltaN0Dot;
}

void Msg_5106_pub_callback(uint8_t * buffer)
{
	Msg_5106 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5106 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5106);
	ROS_DEBUG("Message 0x5106 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5106_pub_initialized == false){
		init_5106(getRosHandle());}
	// Publish the message
	Msg_5106_pub.publish(msgStruct_5106);
	return;
}
