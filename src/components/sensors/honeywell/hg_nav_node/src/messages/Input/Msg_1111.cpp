#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1111.h>
#include <hg_nav_node/trajectory_aiding_validity_discretes_t.h>
hg_nav_node::Msg_1111 msgStruct_1111;

ros::Subscriber Msg_1111_sub;
void init_1111(ros::NodeHandle * n){
	Msg_1111_sub = n->subscribe(MSG_1111_PATH, 5, Msg_1111_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1111_PATH);
	return;
}

void stop_1111(void){
	Msg_1111_sub.shutdown();
	ROS_INFO("0x1111 stopped");
	return;
}

// Msg_1111 to Topic
void convert(Msg_1111 messageIn, hg_nav_node::Msg_1111 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->trajectory_aiding_validity.LATITUDE = messageIn.trajectory_aiding_validity.LATITUDE;
	messageOut->trajectory_aiding_validity.LONGITUDE = messageIn.trajectory_aiding_validity.LONGITUDE;
	messageOut->trajectory_aiding_validity.ALTITUDE = messageIn.trajectory_aiding_validity.ALTITUDE;
	messageOut->trajectory_aiding_validity.BARO_ALTITUDE = messageIn.trajectory_aiding_validity.BARO_ALTITUDE;
	messageOut->trajectory_aiding_validity.TRUE_AIR_SPEED = messageIn.trajectory_aiding_validity.TRUE_AIR_SPEED;
	messageOut->trajectory_aiding_validity.VELOCITY_NORTH = messageIn.trajectory_aiding_validity.VELOCITY_NORTH;
	messageOut->trajectory_aiding_validity.VELOCITY_EAST = messageIn.trajectory_aiding_validity.VELOCITY_EAST;
	messageOut->trajectory_aiding_validity.VELOCITY_DOWN = messageIn.trajectory_aiding_validity.VELOCITY_DOWN;
	messageOut->trajectory_aiding_validity.VELOCITY_X = messageIn.trajectory_aiding_validity.VELOCITY_X;
	messageOut->trajectory_aiding_validity.VELOCITY_Y = messageIn.trajectory_aiding_validity.VELOCITY_Y;
	messageOut->trajectory_aiding_validity.VELOCITY_Z = messageIn.trajectory_aiding_validity.VELOCITY_Z;
	messageOut->trajectory_aiding_validity.ATTITUDE_ROLL = messageIn.trajectory_aiding_validity.ATTITUDE_ROLL;
	messageOut->trajectory_aiding_validity.ATTITUDE_PITCH = messageIn.trajectory_aiding_validity.ATTITUDE_PITCH;
	messageOut->trajectory_aiding_validity.ATTITUDE_HEADING = messageIn.trajectory_aiding_validity.ATTITUDE_HEADING;
	messageOut->trajectory_aiding_validity.ALTITUDE_TYPE = messageIn.trajectory_aiding_validity.ALTITUDE_TYPE;
	messageOut->trajectory_aiding_validity.HEADING_TYPE = messageIn.trajectory_aiding_validity.HEADING_TYPE;
	messageOut->ESpaceTrajectoryTOV = messageIn.ESpaceTrajectoryTOV;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->Altitude_height_above_ellipsoid = messageIn.Altitude_height_above_ellipsoid;
	messageOut->Baro_altitude = messageIn.Baro_altitude;
	messageOut->True_air_speed = messageIn.True_air_speed;
	messageOut->velocity_north = messageIn.velocity_north;
	messageOut->velocity_east = messageIn.velocity_east;
	messageOut->velocity_down = messageIn.velocity_down;
	messageOut->velocity_x = messageIn.velocity_x;
	messageOut->velocity_y = messageIn.velocity_y;
	messageOut->velocity_z = messageIn.velocity_z;
	messageOut->attitude_roll = messageIn.attitude_roll;
	messageOut->attitude_pitch = messageIn.attitude_pitch;
	messageOut->attitude_heading = messageIn.attitude_heading;
}

// Topic to Msg_1111
void convert(hg_nav_node::Msg_1111 messageIn, Msg_1111 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->trajectory_aiding_validity.LATITUDE = messageIn.trajectory_aiding_validity.LATITUDE;
	messageOut->trajectory_aiding_validity.LONGITUDE = messageIn.trajectory_aiding_validity.LONGITUDE;
	messageOut->trajectory_aiding_validity.ALTITUDE = messageIn.trajectory_aiding_validity.ALTITUDE;
	messageOut->trajectory_aiding_validity.BARO_ALTITUDE = messageIn.trajectory_aiding_validity.BARO_ALTITUDE;
	messageOut->trajectory_aiding_validity.TRUE_AIR_SPEED = messageIn.trajectory_aiding_validity.TRUE_AIR_SPEED;
	messageOut->trajectory_aiding_validity.VELOCITY_NORTH = messageIn.trajectory_aiding_validity.VELOCITY_NORTH;
	messageOut->trajectory_aiding_validity.VELOCITY_EAST = messageIn.trajectory_aiding_validity.VELOCITY_EAST;
	messageOut->trajectory_aiding_validity.VELOCITY_DOWN = messageIn.trajectory_aiding_validity.VELOCITY_DOWN;
	messageOut->trajectory_aiding_validity.VELOCITY_X = messageIn.trajectory_aiding_validity.VELOCITY_X;
	messageOut->trajectory_aiding_validity.VELOCITY_Y = messageIn.trajectory_aiding_validity.VELOCITY_Y;
	messageOut->trajectory_aiding_validity.VELOCITY_Z = messageIn.trajectory_aiding_validity.VELOCITY_Z;
	messageOut->trajectory_aiding_validity.ATTITUDE_ROLL = messageIn.trajectory_aiding_validity.ATTITUDE_ROLL;
	messageOut->trajectory_aiding_validity.ATTITUDE_PITCH = messageIn.trajectory_aiding_validity.ATTITUDE_PITCH;
	messageOut->trajectory_aiding_validity.ATTITUDE_HEADING = messageIn.trajectory_aiding_validity.ATTITUDE_HEADING;
	messageOut->trajectory_aiding_validity.ALTITUDE_TYPE = messageIn.trajectory_aiding_validity.ALTITUDE_TYPE;
	messageOut->trajectory_aiding_validity.HEADING_TYPE = messageIn.trajectory_aiding_validity.HEADING_TYPE;
	messageOut->ESpaceTrajectoryTOV = messageIn.ESpaceTrajectoryTOV;
	messageOut->Latitude = messageIn.Latitude;
	messageOut->Longitude = messageIn.Longitude;
	messageOut->Altitude_height_above_ellipsoid = messageIn.Altitude_height_above_ellipsoid;
	messageOut->Baro_altitude = messageIn.Baro_altitude;
	messageOut->True_air_speed = messageIn.True_air_speed;
	messageOut->velocity_north = messageIn.velocity_north;
	messageOut->velocity_east = messageIn.velocity_east;
	messageOut->velocity_down = messageIn.velocity_down;
	messageOut->velocity_x = messageIn.velocity_x;
	messageOut->velocity_y = messageIn.velocity_y;
	messageOut->velocity_z = messageIn.velocity_z;
	messageOut->attitude_roll = messageIn.attitude_roll;
	messageOut->attitude_pitch = messageIn.attitude_pitch;
	messageOut->attitude_heading = messageIn.attitude_heading;
}

void Msg_1111_sub_callback(const hg_nav_node::Msg_1111::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1111 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1111 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1111 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
