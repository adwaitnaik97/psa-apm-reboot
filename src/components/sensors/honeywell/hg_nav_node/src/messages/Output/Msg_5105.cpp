#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_5105.h>
#include <hg_nav_node/gps_mode_table_t.h>
hg_nav_node::Msg_5105 msgStruct_5105;

bool Msg_5105_pub_initialized = false;

ros::Publisher Msg_5105_pub;
void init_5105(ros::NodeHandle * n){
	Msg_5105_pub = n->advertise<hg_nav_node::Msg_5105>(MSG_5105_PATH, 5);
	Msg_5105_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_5105_PATH);
	return;
}

void stop_5105(void){
	Msg_5105_pub.shutdown();
	Msg_5105_pub_initialized = false;
	ROS_INFO("0x5105 stopped");
	return;
}

// Msg_5105 to Topic
void convert(Msg_5105 messageIn, hg_nav_node::Msg_5105 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode.value = static_cast<uint8_t>(messageIn.GPSMode);
	messageOut->ephem_type = messageIn.ephem_type;
	messageOut->satellite_ID = messageIn.satellite_ID;
	messageOut->E5aHealth = messageIn.E5aHealth;
	messageOut->E5bHealth = messageIn.E5bHealth;
	messageOut->E5aDVS = messageIn.E5aDVS;
	messageOut->E5bDVS = messageIn.E5bDVS;
	messageOut->E1bHealth = messageIn.E1bHealth;
	messageOut->E1bDVS = messageIn.E1bDVS;
	messageOut->IODnav = messageIn.IODnav;
	messageOut->SISA_Index = messageIn.SISA_Index;
	messageOut->INAV_Source = messageIn.INAV_Source;
	messageOut->T0e = messageIn.T0e;
	messageOut->T0c = messageIn.T0c;
	messageOut->M0 = messageIn.M0;
	messageOut->DeltaN = messageIn.DeltaN;
	messageOut->Ecc = messageIn.Ecc;
	messageOut->RootA = messageIn.RootA;
	messageOut->I0 = messageIn.I0;
	messageOut->IDot = messageIn.IDot;
	messageOut->Omega0 = messageIn.Omega0;
	messageOut->Omega = messageIn.Omega;
	messageOut->OmegaDot = messageIn.OmegaDot;
	messageOut->Cuc = messageIn.Cuc;
	messageOut->Cus = messageIn.Cus;
	messageOut->Crc = messageIn.Crc;
	messageOut->Crs = messageIn.Crs;
	messageOut->Cic = messageIn.Cic;
	messageOut->Cis = messageIn.Cis;
	messageOut->Af0 = messageIn.Af0;
	messageOut->Af1 = messageIn.Af1;
	messageOut->Af2 = messageIn.Af2;
	messageOut->E1E5aBGD = messageIn.E1E5aBGD;
	messageOut->E1E5bBGD = messageIn.E1E5bBGD;
}

// Topic to Msg_5105
void convert(hg_nav_node::Msg_5105 messageIn, Msg_5105 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->GPSMode = static_cast<gps_mode_table_t>(messageIn.GPSMode.value);
	messageOut->ephem_type = messageIn.ephem_type;
	messageOut->satellite_ID = messageIn.satellite_ID;
	messageOut->E5aHealth = messageIn.E5aHealth;
	messageOut->E5bHealth = messageIn.E5bHealth;
	messageOut->E5aDVS = messageIn.E5aDVS;
	messageOut->E5bDVS = messageIn.E5bDVS;
	messageOut->E1bHealth = messageIn.E1bHealth;
	messageOut->E1bDVS = messageIn.E1bDVS;
	messageOut->IODnav = messageIn.IODnav;
	messageOut->SISA_Index = messageIn.SISA_Index;
	messageOut->INAV_Source = messageIn.INAV_Source;
	messageOut->T0e = messageIn.T0e;
	messageOut->T0c = messageIn.T0c;
	messageOut->M0 = messageIn.M0;
	messageOut->DeltaN = messageIn.DeltaN;
	messageOut->Ecc = messageIn.Ecc;
	messageOut->RootA = messageIn.RootA;
	messageOut->I0 = messageIn.I0;
	messageOut->IDot = messageIn.IDot;
	messageOut->Omega0 = messageIn.Omega0;
	messageOut->Omega = messageIn.Omega;
	messageOut->OmegaDot = messageIn.OmegaDot;
	messageOut->Cuc = messageIn.Cuc;
	messageOut->Cus = messageIn.Cus;
	messageOut->Crc = messageIn.Crc;
	messageOut->Crs = messageIn.Crs;
	messageOut->Cic = messageIn.Cic;
	messageOut->Cis = messageIn.Cis;
	messageOut->Af0 = messageIn.Af0;
	messageOut->Af1 = messageIn.Af1;
	messageOut->Af2 = messageIn.Af2;
	messageOut->E1E5aBGD = messageIn.E1E5aBGD;
	messageOut->E1E5bBGD = messageIn.E1E5bBGD;
}

void Msg_5105_pub_callback(uint8_t * buffer)
{
	Msg_5105 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x5105 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_5105);
	ROS_DEBUG("Message 0x5105 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_5105_pub_initialized == false){
		init_5105(getRosHandle());}
	// Publish the message
	Msg_5105_pub.publish(msgStruct_5105);
	return;
}
