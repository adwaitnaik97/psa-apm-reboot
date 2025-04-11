/*
	Stream Data using FTDI serial to USB converter,
	please make sure you are using super user
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
//Pose transformation
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//YAML configuration
#include "yaml-cpp/yaml.h"

#define PI 3.1415926535897932
#define RAD_TO_DEG 180/PI
#define DEG_TO_RAD PI/180

int main(int argc, char *argv[])
{
	//Init ROS variables
	ros::init(argc,argv, "HGuide_Transformations");

	ros::NodeHandle n;

	//Load configuration file

	if (argc <= 2)
	{
		ROS_INFO("Using Configuration File: \"%s\"",argv[1]);
	}
	else{ROS_ERROR("YAML FILE NOT SPECIFIED!"); return -1;}

	YAML::Node config = YAML::LoadFile(argv[1]);
	int tfNo = 0;
	
	
	if (config["Transformations"]) 
	{
		tfNo = (int) config["Transformations"].size();
		ROS_INFO("%d Transformations found",tfNo);
	}else{ROS_ERROR("WRONG YAML SYNTAX!"); return -2;}

	ROS_INFO("Publishing %d transformations",tfNo);

	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped bodyToNew;
	tf2::Quaternion q;
	bodyToNew.header.frame_id = "HGuide";


	//Publish Transformation
	bodyToNew.header.stamp = ros::Time::now();

	
	for (int i=0;i<tfNo;i++)
	{

		//Translation
		// if not present, set as 0
		if (config["Transformations"][i]["pos"][0] && config["Transformations"][i]["pos"][1] && config["Transformations"][i]["pos"][2])
		{
			bodyToNew.transform.translation.x = config["Transformations"][i]["pos"][0].as<double>();
			bodyToNew.transform.translation.y = config["Transformations"][i]["pos"][1].as<double>();
			bodyToNew.transform.translation.z = config["Transformations"][i]["pos"][2].as<double>();
		}
		else{bodyToNew.transform.translation.x = bodyToNew.transform.translation.y = bodyToNew.transform.translation.z = 0;}

		//Rotation
		// if not present, set as 0
		if (config["Transformations"][i]["rpy"][0] && config["Transformations"][i]["rpy"][1] && config["Transformations"][i]["rpy"][2])
		{
			q.setRPY(config["Transformations"][i]["rpy"][0].as<double>()*DEG_TO_RAD,\
				config["Transformations"][i]["rpy"][1].as<double>()*DEG_TO_RAD,\
				config["Transformations"][i]["rpy"][2].as<double>()*DEG_TO_RAD);
		}
		else{q.setRPY(0,0,0);}

		bodyToNew.transform.rotation.x = q.x();
		bodyToNew.transform.rotation.y = q.y();
		bodyToNew.transform.rotation.z = q.z();
		bodyToNew.transform.rotation.w = q.w();
		
		//Frame Name
		// if not present, transformation cannot be published
		if (config["Transformations"][i]["frameName"])
		{
			bodyToNew.child_frame_id = config["Transformations"][i]["frameName"].as<std::string>();

			br.sendTransform(bodyToNew);		
		}
	}
	
	// no new job for you
	ros::spin();
	
	return 0;
}


