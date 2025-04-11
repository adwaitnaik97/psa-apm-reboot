#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6508.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
#include <hg_nav_node/sat_block_t.h>
hg_nav_node::Msg_6508 msgStruct_6508;

bool Msg_6508_pub_initialized = false;

ros::Publisher Msg_6508_pub;
void init_6508(ros::NodeHandle * n){
	Msg_6508_pub = n->advertise<hg_nav_node::Msg_6508>(MSG_6508_PATH, 5);
	Msg_6508_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6508_PATH);
	return;
}

void stop_6508(void){
	Msg_6508_pub.shutdown();
	Msg_6508_pub_initialized = false;
	ROS_INFO("0x6508 stopped");
	return;
}

// Msg_6508 to Topic
void convert(Msg_6508 messageIn, hg_nav_node::Msg_6508 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

	messageOut->InsGnssSummary.INSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.INSMode);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.GPSMode);

	messageOut->RF1AntSatSearch.TotalSats = messageIn.RF1AntSatSearch.TotalSats;
	messageOut->RF1AntSatSearch.NumGps = messageIn.RF1AntSatSearch.NumGps;
	messageOut->RF1AntSatSearch.NumGlo = messageIn.RF1AntSatSearch.NumGlo;
	messageOut->RF1AntSatSearch.NumGal = messageIn.RF1AntSatSearch.NumGal;
	messageOut->RF1AntSatSearch.NumBds = messageIn.RF1AntSatSearch.NumBds;
	messageOut->RF1AntSatSearch.NumSbas = messageIn.RF1AntSatSearch.NumSbas;
	messageOut->RF1AntSatSearch.NumQzss = messageIn.RF1AntSatSearch.NumQzss;
	messageOut->RF1AntSatSearch.NumIrnss = messageIn.RF1AntSatSearch.NumIrnss;
	messageOut->RF1AntSatSearch.NumLband = messageIn.RF1AntSatSearch.NumLband;
	messageOut->RF1AntSatSearch.TotalSats = messageIn.RF1AntSatSearch.TotalSats;
	messageOut->RF1AntSatSearch.NumGps = messageIn.RF1AntSatSearch.NumGps;
	messageOut->RF1AntSatSearch.NumGlo = messageIn.RF1AntSatSearch.NumGlo;
	messageOut->RF1AntSatSearch.NumGal = messageIn.RF1AntSatSearch.NumGal;
	messageOut->RF1AntSatSearch.NumBds = messageIn.RF1AntSatSearch.NumBds;
	messageOut->RF1AntSatSearch.NumSbas = messageIn.RF1AntSatSearch.NumSbas;
	messageOut->RF1AntSatSearch.NumQzss = messageIn.RF1AntSatSearch.NumQzss;
	messageOut->RF1AntSatSearch.NumIrnss = messageIn.RF1AntSatSearch.NumIrnss;
	messageOut->RF1AntSatSearch.NumLband = messageIn.RF1AntSatSearch.NumLband;

	messageOut->RF1AntSatLock.TotalSats = messageIn.RF1AntSatLock.TotalSats;
	messageOut->RF1AntSatLock.NumGps = messageIn.RF1AntSatLock.NumGps;
	messageOut->RF1AntSatLock.NumGlo = messageIn.RF1AntSatLock.NumGlo;
	messageOut->RF1AntSatLock.NumGal = messageIn.RF1AntSatLock.NumGal;
	messageOut->RF1AntSatLock.NumBds = messageIn.RF1AntSatLock.NumBds;
	messageOut->RF1AntSatLock.NumSbas = messageIn.RF1AntSatLock.NumSbas;
	messageOut->RF1AntSatLock.NumQzss = messageIn.RF1AntSatLock.NumQzss;
	messageOut->RF1AntSatLock.NumIrnss = messageIn.RF1AntSatLock.NumIrnss;
	messageOut->RF1AntSatLock.NumLband = messageIn.RF1AntSatLock.NumLband;
	messageOut->RF1AntSatLock.TotalSats = messageIn.RF1AntSatLock.TotalSats;
	messageOut->RF1AntSatLock.NumGps = messageIn.RF1AntSatLock.NumGps;
	messageOut->RF1AntSatLock.NumGlo = messageIn.RF1AntSatLock.NumGlo;
	messageOut->RF1AntSatLock.NumGal = messageIn.RF1AntSatLock.NumGal;
	messageOut->RF1AntSatLock.NumBds = messageIn.RF1AntSatLock.NumBds;
	messageOut->RF1AntSatLock.NumSbas = messageIn.RF1AntSatLock.NumSbas;
	messageOut->RF1AntSatLock.NumQzss = messageIn.RF1AntSatLock.NumQzss;
	messageOut->RF1AntSatLock.NumIrnss = messageIn.RF1AntSatLock.NumIrnss;
	messageOut->RF1AntSatLock.NumLband = messageIn.RF1AntSatLock.NumLband;

	messageOut->RF2AntSatSearch.TotalSats = messageIn.RF2AntSatSearch.TotalSats;
	messageOut->RF2AntSatSearch.NumGps = messageIn.RF2AntSatSearch.NumGps;
	messageOut->RF2AntSatSearch.NumGlo = messageIn.RF2AntSatSearch.NumGlo;
	messageOut->RF2AntSatSearch.NumGal = messageIn.RF2AntSatSearch.NumGal;
	messageOut->RF2AntSatSearch.NumBds = messageIn.RF2AntSatSearch.NumBds;
	messageOut->RF2AntSatSearch.NumSbas = messageIn.RF2AntSatSearch.NumSbas;
	messageOut->RF2AntSatSearch.NumQzss = messageIn.RF2AntSatSearch.NumQzss;
	messageOut->RF2AntSatSearch.NumIrnss = messageIn.RF2AntSatSearch.NumIrnss;
	messageOut->RF2AntSatSearch.NumLband = messageIn.RF2AntSatSearch.NumLband;
	messageOut->RF2AntSatSearch.TotalSats = messageIn.RF2AntSatSearch.TotalSats;
	messageOut->RF2AntSatSearch.NumGps = messageIn.RF2AntSatSearch.NumGps;
	messageOut->RF2AntSatSearch.NumGlo = messageIn.RF2AntSatSearch.NumGlo;
	messageOut->RF2AntSatSearch.NumGal = messageIn.RF2AntSatSearch.NumGal;
	messageOut->RF2AntSatSearch.NumBds = messageIn.RF2AntSatSearch.NumBds;
	messageOut->RF2AntSatSearch.NumSbas = messageIn.RF2AntSatSearch.NumSbas;
	messageOut->RF2AntSatSearch.NumQzss = messageIn.RF2AntSatSearch.NumQzss;
	messageOut->RF2AntSatSearch.NumIrnss = messageIn.RF2AntSatSearch.NumIrnss;
	messageOut->RF2AntSatSearch.NumLband = messageIn.RF2AntSatSearch.NumLband;

	messageOut->RF2AntSatLock.TotalSats = messageIn.RF2AntSatLock.TotalSats;
	messageOut->RF2AntSatLock.NumGps = messageIn.RF2AntSatLock.NumGps;
	messageOut->RF2AntSatLock.NumGlo = messageIn.RF2AntSatLock.NumGlo;
	messageOut->RF2AntSatLock.NumGal = messageIn.RF2AntSatLock.NumGal;
	messageOut->RF2AntSatLock.NumBds = messageIn.RF2AntSatLock.NumBds;
	messageOut->RF2AntSatLock.NumSbas = messageIn.RF2AntSatLock.NumSbas;
	messageOut->RF2AntSatLock.NumQzss = messageIn.RF2AntSatLock.NumQzss;
	messageOut->RF2AntSatLock.NumIrnss = messageIn.RF2AntSatLock.NumIrnss;
	messageOut->RF2AntSatLock.NumLband = messageIn.RF2AntSatLock.NumLband;
	messageOut->RF2AntSatLock.TotalSats = messageIn.RF2AntSatLock.TotalSats;
	messageOut->RF2AntSatLock.NumGps = messageIn.RF2AntSatLock.NumGps;
	messageOut->RF2AntSatLock.NumGlo = messageIn.RF2AntSatLock.NumGlo;
	messageOut->RF2AntSatLock.NumGal = messageIn.RF2AntSatLock.NumGal;
	messageOut->RF2AntSatLock.NumBds = messageIn.RF2AntSatLock.NumBds;
	messageOut->RF2AntSatLock.NumSbas = messageIn.RF2AntSatLock.NumSbas;
	messageOut->RF2AntSatLock.NumQzss = messageIn.RF2AntSatLock.NumQzss;
	messageOut->RF2AntSatLock.NumIrnss = messageIn.RF2AntSatLock.NumIrnss;
	messageOut->RF2AntSatLock.NumLband = messageIn.RF2AntSatLock.NumLband;
}

// Topic to Msg_6508
void convert(hg_nav_node::Msg_6508 messageIn, Msg_6508 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;
	messageOut->gps_week = messageIn.gps_week;

	messageOut->InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(messageIn.InsGnssSummary.INSMode.value);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(messageIn.InsGnssSummary.GPSMode.value);

	messageOut->RF1AntSatSearch.TotalSats = messageIn.RF1AntSatSearch.TotalSats;
	messageOut->RF1AntSatSearch.NumGps = messageIn.RF1AntSatSearch.NumGps;
	messageOut->RF1AntSatSearch.NumGlo = messageIn.RF1AntSatSearch.NumGlo;
	messageOut->RF1AntSatSearch.NumGal = messageIn.RF1AntSatSearch.NumGal;
	messageOut->RF1AntSatSearch.NumBds = messageIn.RF1AntSatSearch.NumBds;
	messageOut->RF1AntSatSearch.NumSbas = messageIn.RF1AntSatSearch.NumSbas;
	messageOut->RF1AntSatSearch.NumQzss = messageIn.RF1AntSatSearch.NumQzss;
	messageOut->RF1AntSatSearch.NumIrnss = messageIn.RF1AntSatSearch.NumIrnss;
	messageOut->RF1AntSatSearch.NumLband = messageIn.RF1AntSatSearch.NumLband;
	messageOut->RF1AntSatSearch.TotalSats = messageIn.RF1AntSatSearch.TotalSats;
	messageOut->RF1AntSatSearch.NumGps = messageIn.RF1AntSatSearch.NumGps;
	messageOut->RF1AntSatSearch.NumGlo = messageIn.RF1AntSatSearch.NumGlo;
	messageOut->RF1AntSatSearch.NumGal = messageIn.RF1AntSatSearch.NumGal;
	messageOut->RF1AntSatSearch.NumBds = messageIn.RF1AntSatSearch.NumBds;
	messageOut->RF1AntSatSearch.NumSbas = messageIn.RF1AntSatSearch.NumSbas;
	messageOut->RF1AntSatSearch.NumQzss = messageIn.RF1AntSatSearch.NumQzss;
	messageOut->RF1AntSatSearch.NumIrnss = messageIn.RF1AntSatSearch.NumIrnss;
	messageOut->RF1AntSatSearch.NumLband = messageIn.RF1AntSatSearch.NumLband;

	messageOut->RF1AntSatLock.TotalSats = messageIn.RF1AntSatLock.TotalSats;
	messageOut->RF1AntSatLock.NumGps = messageIn.RF1AntSatLock.NumGps;
	messageOut->RF1AntSatLock.NumGlo = messageIn.RF1AntSatLock.NumGlo;
	messageOut->RF1AntSatLock.NumGal = messageIn.RF1AntSatLock.NumGal;
	messageOut->RF1AntSatLock.NumBds = messageIn.RF1AntSatLock.NumBds;
	messageOut->RF1AntSatLock.NumSbas = messageIn.RF1AntSatLock.NumSbas;
	messageOut->RF1AntSatLock.NumQzss = messageIn.RF1AntSatLock.NumQzss;
	messageOut->RF1AntSatLock.NumIrnss = messageIn.RF1AntSatLock.NumIrnss;
	messageOut->RF1AntSatLock.NumLband = messageIn.RF1AntSatLock.NumLband;
	messageOut->RF1AntSatLock.TotalSats = messageIn.RF1AntSatLock.TotalSats;
	messageOut->RF1AntSatLock.NumGps = messageIn.RF1AntSatLock.NumGps;
	messageOut->RF1AntSatLock.NumGlo = messageIn.RF1AntSatLock.NumGlo;
	messageOut->RF1AntSatLock.NumGal = messageIn.RF1AntSatLock.NumGal;
	messageOut->RF1AntSatLock.NumBds = messageIn.RF1AntSatLock.NumBds;
	messageOut->RF1AntSatLock.NumSbas = messageIn.RF1AntSatLock.NumSbas;
	messageOut->RF1AntSatLock.NumQzss = messageIn.RF1AntSatLock.NumQzss;
	messageOut->RF1AntSatLock.NumIrnss = messageIn.RF1AntSatLock.NumIrnss;
	messageOut->RF1AntSatLock.NumLband = messageIn.RF1AntSatLock.NumLband;

	messageOut->RF2AntSatSearch.TotalSats = messageIn.RF2AntSatSearch.TotalSats;
	messageOut->RF2AntSatSearch.NumGps = messageIn.RF2AntSatSearch.NumGps;
	messageOut->RF2AntSatSearch.NumGlo = messageIn.RF2AntSatSearch.NumGlo;
	messageOut->RF2AntSatSearch.NumGal = messageIn.RF2AntSatSearch.NumGal;
	messageOut->RF2AntSatSearch.NumBds = messageIn.RF2AntSatSearch.NumBds;
	messageOut->RF2AntSatSearch.NumSbas = messageIn.RF2AntSatSearch.NumSbas;
	messageOut->RF2AntSatSearch.NumQzss = messageIn.RF2AntSatSearch.NumQzss;
	messageOut->RF2AntSatSearch.NumIrnss = messageIn.RF2AntSatSearch.NumIrnss;
	messageOut->RF2AntSatSearch.NumLband = messageIn.RF2AntSatSearch.NumLband;
	messageOut->RF2AntSatSearch.TotalSats = messageIn.RF2AntSatSearch.TotalSats;
	messageOut->RF2AntSatSearch.NumGps = messageIn.RF2AntSatSearch.NumGps;
	messageOut->RF2AntSatSearch.NumGlo = messageIn.RF2AntSatSearch.NumGlo;
	messageOut->RF2AntSatSearch.NumGal = messageIn.RF2AntSatSearch.NumGal;
	messageOut->RF2AntSatSearch.NumBds = messageIn.RF2AntSatSearch.NumBds;
	messageOut->RF2AntSatSearch.NumSbas = messageIn.RF2AntSatSearch.NumSbas;
	messageOut->RF2AntSatSearch.NumQzss = messageIn.RF2AntSatSearch.NumQzss;
	messageOut->RF2AntSatSearch.NumIrnss = messageIn.RF2AntSatSearch.NumIrnss;
	messageOut->RF2AntSatSearch.NumLband = messageIn.RF2AntSatSearch.NumLband;

	messageOut->RF2AntSatLock.TotalSats = messageIn.RF2AntSatLock.TotalSats;
	messageOut->RF2AntSatLock.NumGps = messageIn.RF2AntSatLock.NumGps;
	messageOut->RF2AntSatLock.NumGlo = messageIn.RF2AntSatLock.NumGlo;
	messageOut->RF2AntSatLock.NumGal = messageIn.RF2AntSatLock.NumGal;
	messageOut->RF2AntSatLock.NumBds = messageIn.RF2AntSatLock.NumBds;
	messageOut->RF2AntSatLock.NumSbas = messageIn.RF2AntSatLock.NumSbas;
	messageOut->RF2AntSatLock.NumQzss = messageIn.RF2AntSatLock.NumQzss;
	messageOut->RF2AntSatLock.NumIrnss = messageIn.RF2AntSatLock.NumIrnss;
	messageOut->RF2AntSatLock.NumLband = messageIn.RF2AntSatLock.NumLband;
	messageOut->RF2AntSatLock.TotalSats = messageIn.RF2AntSatLock.TotalSats;
	messageOut->RF2AntSatLock.NumGps = messageIn.RF2AntSatLock.NumGps;
	messageOut->RF2AntSatLock.NumGlo = messageIn.RF2AntSatLock.NumGlo;
	messageOut->RF2AntSatLock.NumGal = messageIn.RF2AntSatLock.NumGal;
	messageOut->RF2AntSatLock.NumBds = messageIn.RF2AntSatLock.NumBds;
	messageOut->RF2AntSatLock.NumSbas = messageIn.RF2AntSatLock.NumSbas;
	messageOut->RF2AntSatLock.NumQzss = messageIn.RF2AntSatLock.NumQzss;
	messageOut->RF2AntSatLock.NumIrnss = messageIn.RF2AntSatLock.NumIrnss;
	messageOut->RF2AntSatLock.NumLband = messageIn.RF2AntSatLock.NumLband;
}

void Msg_6508_pub_callback(uint8_t * buffer)
{
	Msg_6508 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6508 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6508);
	ROS_DEBUG("Message 0x6508 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6508_pub_initialized == false){
		init_6508(getRosHandle());}
	// Publish the message
	Msg_6508_pub.publish(msgStruct_6508);
	return;
}
