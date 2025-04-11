/*
	Enables projection of navigaton unit in rviz SW
*/

/*
HONEYWELL hereby grants to you a, perpetual, free of charge, worldwide, irrevocable, non-exclusive license to use, copy, modify, merge,
publish, distribute, sublicense the software and associated documentation (the Software), subject to the following conditions:

YOU AGREE THAT YOU ASSUME ALL THE RESPONSIBILITY AND RISK FOR YOUR USE OF THE SOFTWARE AND THE RESULTS AND PERFORMANCE THEREOF.
THE SOFTWARE IS PROVIDED TO YOU ON AN AS IS AND AS AVAILABLE BASIS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
WITHOUT LIMITATION ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

IN NO EVENT WILL HONEYWELL BE LIABLE TO YOU FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, CONSEQUENTIAL, EXEMPLARY OR PUNITIVE DAMAGES,
INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOST DATA, LOST PROFITS, LOSS OF GOODWILL, LOST REVENUE, SERVICE INTERRUPTION, DEVICE DAMAGE
OR SYSTEM FAILURE, UNDER ANY THEORY OF LIABILITY, INCLUDING, WITHOUT LIMITATION CONTRACT OR TORT, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE.

All technology from the United States is subject to export regulations. This software is related to a device that has a United States
Export Commodity Classification of ECCN 7A994 with associated country chart control code of AT1. This generally will not require a license
to be exported or re-exported. However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern,
or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include "ros/ros.h"

//Custom messages
#include "api_to_topics.h"

#define PI 3.1415926535897932
#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI/180

#include <visualization_msgs/Marker.h>
visualization_msgs::Marker NedVelMarker;
visualization_msgs::Marker InertialMarker;

//ACK/NAK Message callback
void hgAckCallback(const hg_nav_node::Msg_20FF::ConstPtr& Message)
{
	if (Message->Ack)
		ROS_INFO("ACK for 0x%.4x received - total valid messages: %d",Message->InputMessageID,Message->NoOfValidMessagesSincePowerUp);
	else
		ROS_ERROR("NAK for 0x%.4x received - total valid messages: %d",Message->InputMessageID,Message->NoOfValidMessagesSincePowerUp);

	return;
}

void hgNedVelocityCallback(const hg_nav_node::Msg_6504::ConstPtr& Message)
{
	char buf[128]={0};
	NedVelMarker.header.stamp = ros::Time::now();
	sprintf(buf,"Velocity:\n  North:  % .4f\n  East:   % .4f\n  Down:  % .4f",Message->NorthVelocity,Message->EastVelocity,Message->DownVelocity);
	NedVelMarker.text=buf;
	return;
}

void hgInertialDataCallback(const hg_nav_node::Msg_2311::ConstPtr& Message)
{
	char buf[128]={0};
	InertialMarker.header.stamp = ros::Time::now();
	sprintf(buf,"Delta Theta:\n  x:  % .4f\n  y:  % .4f\n  z:  % .4f\nDelta Velocity:\n  x:  % .4f\n  y:  % .4f\n  z:  % .4f",Message->delta_theta_x,Message->delta_theta_y,Message->delta_theta_z,Message->delta_velocity_x,Message->delta_velocity_y,Message->delta_velocity_z);
	InertialMarker.text=buf;
	return;
}

int main(int argc, char *argv[])
{
	//init ROS variables
	ros::init(argc,argv, "HgDemoAiding");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);
	
	//Marker with data
	ros::Publisher NedVelMarker_pub = n.advertise<visualization_msgs::Marker>("NED_Velocity", 1);	
	ros::Publisher InertialMarker_pub = n.advertise<visualization_msgs::Marker>("Inertial_Data", 1);	
	
	NedVelMarker.header.frame_id = "LocalVertical";
	NedVelMarker.header.stamp = ros::Time::now();
	NedVelMarker.ns = "basic_shapes";
	NedVelMarker.id = 1;
	NedVelMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	NedVelMarker.action = visualization_msgs::Marker::ADD;

	NedVelMarker.text = "Data";

	NedVelMarker.scale.x = 0.3;
	NedVelMarker.scale.y = 0.3;
	NedVelMarker.scale.z = 0.1;

	NedVelMarker.color.r = 0.0f;
	NedVelMarker.color.g = 1.0f;
	NedVelMarker.color.b = 0.0f;
	NedVelMarker.color.a = 1.0;

	NedVelMarker.pose.position.x = 0.5;
	NedVelMarker.pose.position.y = 0.5;
	NedVelMarker.pose.position.z = 0.5;

	InertialMarker.header.frame_id = "LocalVertical";
	InertialMarker.header.stamp = ros::Time::now();
	InertialMarker.ns = "basic_shapes";
	InertialMarker.id = 2;
	InertialMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	InertialMarker.action = visualization_msgs::Marker::ADD;

	InertialMarker.text = "Data";

	InertialMarker.scale.x = 0.3;
	InertialMarker.scale.y = 0.3;
	InertialMarker.scale.z = 0.1;

	InertialMarker.color.r = 0.0f;
	InertialMarker.color.g = 1.0f;
	InertialMarker.color.b = 0.0f;
	InertialMarker.color.a = 1.0;

	InertialMarker.pose.position.x = -0.5;
	InertialMarker.pose.position.y = 0.5;
	InertialMarker.pose.position.z = 0.5;


	ros::Publisher HgNavIn_pub = n.advertise<hg_nav_node::Msg_1401> (MSG_1401_PATH, 5); // publish on predefined topic
	hg_nav_node::Msg_1401 NavInputMessage;

	if (argc >= 4) //lat lon lat
	{
		ROS_INFO("Position Received:");
		NavInputMessage.Latitude=atof(argv[1])*DEG_TO_RAD; // [rad] latitude position
		NavInputMessage.Longitude=atof(argv[2])*DEG_TO_RAD; // [rad] longitude position
		NavInputMessage.AltitudeHeightAboveEllipsoid=atof(argv[3]); //[m] altitude above ellipsoid
	}
	else
	{
		ROS_INFO("Using default position:");
		NavInputMessage.Latitude=45.0f*DEG_TO_RAD; // [rad] latitude position
		NavInputMessage.Longitude=-93.0f*DEG_TO_RAD; // [rad] longitude position
		NavInputMessage.AltitudeHeightAboveEllipsoid=252.0f; //[m] altitude above ellipsoid




	}
	ROS_INFO("\n\tLatitude:\t%.2f\n\tLongitude:\t%.2f\n\tAltitude:\t%.2f",NavInputMessage.Latitude,NavInputMessage.Longitude,NavInputMessage.AltitudeHeightAboveEllipsoid);

	NavInputMessage.RequestACKNAKReply = true;
	NavInputMessage.AttitudeValidity = true;
	NavInputMessage.AttitudeTimeReferenceMode = true;	
	NavInputMessage.AttitudeCoordinateFrame = true;

	NavInputMessage.PositionValidity= true; // 0 = invalid | 1 = valid
	NavInputMessage.PositionTimeReferenceMode = 1; // 0 = gps time | 1 = Message Receipt Timestamp
	NavInputMessage.PositionStdvValidity = 1; // 0 = invalid | 1 = valid

	NavInputMessage.PositionStdvNorth=0.01f; // [m] STDV North
	NavInputMessage.PositionStdvEast=0.01f; // [m] STDV East
	NavInputMessage.PositionStdvDown=0.01f; // [m] STDV Down

	NavInputMessage.EulerAnglesStdvRoll=1.0f*DEG_TO_RAD; // [rad] STDV Roll
	NavInputMessage.EulerAnglesStdvPitch=1.0f*DEG_TO_RAD; // [rad] STDV Pitch
	NavInputMessage.EulerAnglesStdvTrueHeading=0.01f*DEG_TO_RAD; // [rad] STDV True Heading

	//Barometric altitude aiding
	ros::Publisher HgBaroAid_pub = n.advertise<hg_nav_node::Msg_1101> (MSG_1101_PATH, 5); // publish on predefined topic
	hg_nav_node::Msg_1101 BaroAltInputMessage;
	BaroAltInputMessage.BarometricAltitudeTov = 0;
	BaroAltInputMessage.TimeReferenceMode=1;
	BaroAltInputMessage.BarometricAltitudeValidity = 1;
	BaroAltInputMessage.BarometricAltitudeMSLGeoid=NavInputMessage.AltitudeHeightAboveEllipsoid;

	//init Subscriber nodes
	ros::Subscriber HgAck_sub = n.subscribe(MSG_20FF_PATH, 10,hgAckCallback);
	ros::Subscriber HgNedVelocity_sub = n.subscribe(MSG_6504_PATH, 1,hgNedVelocityCallback);
	ros::Subscriber HgInertialData_sub = n.subscribe(MSG_2311_PATH, 1,hgInertialDataCallback);

	//Send desired settings to Master control
	ROS_INFO("Configuring Device");
	//Wait for the serial publisher to start
	while(NedVelMarker.text=="Data")
	{
		HgNavIn_pub.publish(NavInputMessage);
		ros::spinOnce();
		ros::Duration(0.5).sleep();
	}



	ROS_INFO("Aiding navigation...");

	int counter=10;
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

		if (!counter)
		{
			HgBaroAid_pub.publish(BaroAltInputMessage); //correct height
			counter = 10;
		}
		else {counter--;}
		
		//Update the markers
		InertialMarker_pub.publish(InertialMarker);
		NedVelMarker_pub.publish(NedVelMarker);
	}

	return 0;
}
