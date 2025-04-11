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

All technology that leaves the United States is subject to export regulations. This manual contains technology that has an Export Commodity Classification of EAR99.
This technology generally will not require a license to be exported or reexported. 
However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern, or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include "ros/ros.h"
#include "ros/master.h"

//Standard Messages
#include <sensor_msgs/Imu.h>

//turtlesim messages
#include "geometry_msgs/Twist.h"

ros::Publisher turtleTwist_pub;

//sensor_msgs::Imu callback
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	// twist message for turtle
	geometry_msgs::Twist turtleTwist;

	//X,Y,Z arranged to work with unit with X axis facing up
	turtleTwist.angular.z = msg->angular_velocity.x;
	turtleTwist.angular.y = msg->angular_velocity.y;
	turtleTwist.angular.x = msg->angular_velocity.z;

	turtleTwist.linear.z = msg->linear_acceleration.x;
	turtleTwist.linear.x = msg->linear_acceleration.y;
	turtleTwist.linear.y = msg->linear_acceleration.z;
	
	turtleTwist_pub.publish(turtleTwist);
	return;
}




int main(int argc, char *argv[])
{
	//init ROS variables
	ros::init(argc,argv, "HgImuToTurtle");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(600);
		
	//find the correct topic name
	char topicName[128]={0};
	ros::master::V_TopicInfo master_topics;

	// look for "/Imu" in topics with 10s timeout.
	int timeout = 1000;
	do
	{	
		ros::Duration(0.01).sleep();
		//get list of topics
		ros::master::getTopics(master_topics); 
		//cycle through topics to find correct
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it!=master_topics.end();it++)
		{
			const ros::master::TopicInfo& info = *it;
			if (info.name.find("/Imu") != std::string::npos)
			{
				strcpy(topicName,info.name.c_str());
				ROS_INFO("Using %s topic",topicName);
				break;
			}
				
		}
	timeout--;
	}while(topicName[0]==0 && timeout);
	
	if (topicName[0] == 0)
	{
		ROS_ERROR("Topic not found!");
		return -1;	
	}

	//init Subscriber node
	ros::Subscriber HgControl_sub = n.subscribe(topicName,5,imuCallback);
	
	//publish data to Turtlesim
	turtleTwist_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);

	ROS_INFO("Sending data to turtle...");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
