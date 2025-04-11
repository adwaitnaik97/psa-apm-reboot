#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6722.h>
#include <hg_nav_node/dvl_track_status_t.h>
hg_nav_node::Msg_6722 msgStruct_6722;

bool Msg_6722_pub_initialized = false;

ros::Publisher Msg_6722_pub;
void init_6722(ros::NodeHandle * n){
	Msg_6722_pub = n->advertise<hg_nav_node::Msg_6722>(MSG_6722_PATH, 5);
	Msg_6722_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6722_PATH);
	return;
}

void stop_6722(void){
	Msg_6722_pub.shutdown();
	Msg_6722_pub_initialized = false;
	ROS_INFO("0x6722 stopped");
	return;
}

// Msg_6722 to Topic
void convert(Msg_6722 messageIn, hg_nav_node::Msg_6722 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->serialNumber = messageIn.serialNumber;
	messageOut->version = messageIn.version;
	messageOut->offsetOfData = messageIn.offsetOfData;
	messageOut->year = messageIn.year;
	messageOut->month = messageIn.month;
	messageOut->day = messageIn.day;
	messageOut->hour = messageIn.hour;
	messageOut->minute = messageIn.minute;
	messageOut->seconds = messageIn.seconds;
	messageOut->microseconds = messageIn.microseconds;
	messageOut->numOfBeams = messageIn.numOfBeams;
	messageOut->error = messageIn.error;

	messageOut->status.beam1_velocityValid = messageIn.status.beam1_velocityValid;
	messageOut->status.beam2_velocityValid = messageIn.status.beam2_velocityValid;
	messageOut->status.beam3_velocityValid = messageIn.status.beam3_velocityValid;
	messageOut->status.beam4_velocityValid = messageIn.status.beam4_velocityValid;
	messageOut->status.beam1_distanceValid = messageIn.status.beam1_distanceValid;
	messageOut->status.beam2_distanceValid = messageIn.status.beam2_distanceValid;
	messageOut->status.beam3_distanceValid = messageIn.status.beam3_distanceValid;
	messageOut->status.beam4_distanceValid = messageIn.status.beam4_distanceValid;
	messageOut->status.beam1_fomValid = messageIn.status.beam1_fomValid;
	messageOut->status.beam2_fomValid = messageIn.status.beam2_fomValid;
	messageOut->status.beam3_fomValid = messageIn.status.beam3_fomValid;
	messageOut->status.beam4_fomValid = messageIn.status.beam4_fomValid;
	messageOut->status.x_velocityValid = messageIn.status.x_velocityValid;
	messageOut->status.y_velocityValid = messageIn.status.y_velocityValid;
	messageOut->status.z1_velocityValid = messageIn.status.z1_velocityValid;
	messageOut->status.z2_velocityValid = messageIn.status.z2_velocityValid;
	messageOut->status.x_fomValid = messageIn.status.x_fomValid;
	messageOut->status.y_fomValid = messageIn.status.y_fomValid;
	messageOut->status.z1_fomValid = messageIn.status.z1_fomValid;
	messageOut->status.z2_fomValid = messageIn.status.z2_fomValid;
	messageOut->status.cpu_load = messageIn.status.cpu_load;
	messageOut->status.wakeup_state = messageIn.status.wakeup_state;
	messageOut->speedOfSound = messageIn.speedOfSound;
	messageOut->temperature = messageIn.temperature;
	messageOut->pressure = messageIn.pressure;
	messageOut->velBeam0 = messageIn.velBeam0;
	messageOut->velBeam1 = messageIn.velBeam1;
	messageOut->velBeam2 = messageIn.velBeam2;
	messageOut->velBeam3 = messageIn.velBeam3;
	messageOut->distBeam0 = messageIn.distBeam0;
	messageOut->distBeam1 = messageIn.distBeam1;
	messageOut->distBeam2 = messageIn.distBeam2;
	messageOut->distBeam3 = messageIn.distBeam3;
	messageOut->fomBeam0 = messageIn.fomBeam0;
	messageOut->fomBeam1 = messageIn.fomBeam1;
	messageOut->fomBeam2 = messageIn.fomBeam2;
	messageOut->fomBeam3 = messageIn.fomBeam3;
	messageOut->dt1Beam0 = messageIn.dt1Beam0;
	messageOut->dt1Beam1 = messageIn.dt1Beam1;
	messageOut->dt1Beam2 = messageIn.dt1Beam2;
	messageOut->dt1Beam3 = messageIn.dt1Beam3;
	messageOut->dt2Beam0 = messageIn.dt2Beam0;
	messageOut->dt2Beam1 = messageIn.dt2Beam1;
	messageOut->dt2Beam2 = messageIn.dt2Beam2;
	messageOut->dt2Beam3 = messageIn.dt2Beam3;
	messageOut->timeVelEstBeam0 = messageIn.timeVelEstBeam0;
	messageOut->timeVelEstBeam1 = messageIn.timeVelEstBeam1;
	messageOut->timeVelEstBeam2 = messageIn.timeVelEstBeam2;
	messageOut->timeVelEstBeam3 = messageIn.timeVelEstBeam3;
	messageOut->velX = messageIn.velX;
	messageOut->velY = messageIn.velY;
	messageOut->velZ1 = messageIn.velZ1;
	messageOut->velZ2 = messageIn.velZ2;
	messageOut->fomX = messageIn.fomX;
	messageOut->fomY = messageIn.fomY;
	messageOut->fomZ1 = messageIn.fomZ1;
	messageOut->fomZ2 = messageIn.fomZ2;
	messageOut->dt1X = messageIn.dt1X;
	messageOut->dt1Y = messageIn.dt1Y;
	messageOut->dt1Z1 = messageIn.dt1Z1;
	messageOut->dt1Z2 = messageIn.dt1Z2;
	messageOut->dt2X = messageIn.dt2X;
	messageOut->dt2Y = messageIn.dt2Y;
	messageOut->dt2Z1 = messageIn.dt2Z1;
	messageOut->dt2Z2 = messageIn.dt2Z2;
	messageOut->timeVelEstX = messageIn.timeVelEstX;
	messageOut->timeVelEstY = messageIn.timeVelEstY;
	messageOut->timeVelEstZ1 = messageIn.timeVelEstZ1;
	messageOut->timeVelEstZ2 = messageIn.timeVelEstZ2;
}

// Topic to Msg_6722
void convert(hg_nav_node::Msg_6722 messageIn, Msg_6722 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->serialNumber = messageIn.serialNumber;
	messageOut->version = messageIn.version;
	messageOut->offsetOfData = messageIn.offsetOfData;
	messageOut->year = messageIn.year;
	messageOut->month = messageIn.month;
	messageOut->day = messageIn.day;
	messageOut->hour = messageIn.hour;
	messageOut->minute = messageIn.minute;
	messageOut->seconds = messageIn.seconds;
	messageOut->microseconds = messageIn.microseconds;
	messageOut->numOfBeams = messageIn.numOfBeams;
	messageOut->error = messageIn.error;

	messageOut->status.beam1_velocityValid = messageIn.status.beam1_velocityValid;
	messageOut->status.beam2_velocityValid = messageIn.status.beam2_velocityValid;
	messageOut->status.beam3_velocityValid = messageIn.status.beam3_velocityValid;
	messageOut->status.beam4_velocityValid = messageIn.status.beam4_velocityValid;
	messageOut->status.beam1_distanceValid = messageIn.status.beam1_distanceValid;
	messageOut->status.beam2_distanceValid = messageIn.status.beam2_distanceValid;
	messageOut->status.beam3_distanceValid = messageIn.status.beam3_distanceValid;
	messageOut->status.beam4_distanceValid = messageIn.status.beam4_distanceValid;
	messageOut->status.beam1_fomValid = messageIn.status.beam1_fomValid;
	messageOut->status.beam2_fomValid = messageIn.status.beam2_fomValid;
	messageOut->status.beam3_fomValid = messageIn.status.beam3_fomValid;
	messageOut->status.beam4_fomValid = messageIn.status.beam4_fomValid;
	messageOut->status.x_velocityValid = messageIn.status.x_velocityValid;
	messageOut->status.y_velocityValid = messageIn.status.y_velocityValid;
	messageOut->status.z1_velocityValid = messageIn.status.z1_velocityValid;
	messageOut->status.z2_velocityValid = messageIn.status.z2_velocityValid;
	messageOut->status.x_fomValid = messageIn.status.x_fomValid;
	messageOut->status.y_fomValid = messageIn.status.y_fomValid;
	messageOut->status.z1_fomValid = messageIn.status.z1_fomValid;
	messageOut->status.z2_fomValid = messageIn.status.z2_fomValid;
	messageOut->status.cpu_load = messageIn.status.cpu_load;
	messageOut->status.wakeup_state = messageIn.status.wakeup_state;
	messageOut->speedOfSound = messageIn.speedOfSound;
	messageOut->temperature = messageIn.temperature;
	messageOut->pressure = messageIn.pressure;
	messageOut->velBeam0 = messageIn.velBeam0;
	messageOut->velBeam1 = messageIn.velBeam1;
	messageOut->velBeam2 = messageIn.velBeam2;
	messageOut->velBeam3 = messageIn.velBeam3;
	messageOut->distBeam0 = messageIn.distBeam0;
	messageOut->distBeam1 = messageIn.distBeam1;
	messageOut->distBeam2 = messageIn.distBeam2;
	messageOut->distBeam3 = messageIn.distBeam3;
	messageOut->fomBeam0 = messageIn.fomBeam0;
	messageOut->fomBeam1 = messageIn.fomBeam1;
	messageOut->fomBeam2 = messageIn.fomBeam2;
	messageOut->fomBeam3 = messageIn.fomBeam3;
	messageOut->dt1Beam0 = messageIn.dt1Beam0;
	messageOut->dt1Beam1 = messageIn.dt1Beam1;
	messageOut->dt1Beam2 = messageIn.dt1Beam2;
	messageOut->dt1Beam3 = messageIn.dt1Beam3;
	messageOut->dt2Beam0 = messageIn.dt2Beam0;
	messageOut->dt2Beam1 = messageIn.dt2Beam1;
	messageOut->dt2Beam2 = messageIn.dt2Beam2;
	messageOut->dt2Beam3 = messageIn.dt2Beam3;
	messageOut->timeVelEstBeam0 = messageIn.timeVelEstBeam0;
	messageOut->timeVelEstBeam1 = messageIn.timeVelEstBeam1;
	messageOut->timeVelEstBeam2 = messageIn.timeVelEstBeam2;
	messageOut->timeVelEstBeam3 = messageIn.timeVelEstBeam3;
	messageOut->velX = messageIn.velX;
	messageOut->velY = messageIn.velY;
	messageOut->velZ1 = messageIn.velZ1;
	messageOut->velZ2 = messageIn.velZ2;
	messageOut->fomX = messageIn.fomX;
	messageOut->fomY = messageIn.fomY;
	messageOut->fomZ1 = messageIn.fomZ1;
	messageOut->fomZ2 = messageIn.fomZ2;
	messageOut->dt1X = messageIn.dt1X;
	messageOut->dt1Y = messageIn.dt1Y;
	messageOut->dt1Z1 = messageIn.dt1Z1;
	messageOut->dt1Z2 = messageIn.dt1Z2;
	messageOut->dt2X = messageIn.dt2X;
	messageOut->dt2Y = messageIn.dt2Y;
	messageOut->dt2Z1 = messageIn.dt2Z1;
	messageOut->dt2Z2 = messageIn.dt2Z2;
	messageOut->timeVelEstX = messageIn.timeVelEstX;
	messageOut->timeVelEstY = messageIn.timeVelEstY;
	messageOut->timeVelEstZ1 = messageIn.timeVelEstZ1;
	messageOut->timeVelEstZ2 = messageIn.timeVelEstZ2;
}

void Msg_6722_pub_callback(uint8_t * buffer)
{
	Msg_6722 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6722 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6722);
	ROS_DEBUG("Message 0x6722 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6722_pub_initialized == false){
		init_6722(getRosHandle());}
	// Publish the message
	Msg_6722_pub.publish(msgStruct_6722);
	return;
}
