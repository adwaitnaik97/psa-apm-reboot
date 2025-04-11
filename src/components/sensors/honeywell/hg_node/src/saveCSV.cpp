/*
	Simple example to read and save data from HG1120 Inertial Data publisher
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

All technology that leaves the United States is subject to export regulations. This manual contains technology that has an Export Commodity Classification of EAR99.
This technology generally will not require a license to be exported or reexported. 
However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern, or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include "ros/ros.h"
#include "ros/master.h"

//Custom messages
#include "api_to_topics.h"


//pointer to csv file
FILE * file_ctrl;
FILE * file_inertial;
bool file_crtl_open = false;
bool file_inertial_open = false;

void printControl(float rateX,float rateY,float rateZ,float accelX,float accelY,float accelZ)
{
	if (file_crtl_open == false) //open file if not opened
	{
		file_ctrl = fopen("ControlData.csv","a");
		if (file_ctrl == NULL)
		{
			ROS_INFO("Failed to open file");
			return;
		}
		fprintf(file_ctrl,"AngularRateX,AngularRateY,AngularRateZ,LinearAccelerationX,LinearAccelerationY,LinearAccelerationZ");
		
		file_crtl_open = true;
	}
	fprintf(file_ctrl,"\n%.6f,%.6f,%.6f,%.6f,%.6f,%.6f", rateX, rateY, rateZ, accelX, accelY, accelZ);
	return;
}
void printInertial(float dt_x,float dt_y,float dt_z,float dv_x,float dv_y,float dv_z)
{
	if (file_inertial_open == false) //open file if not opened
	{
		file_inertial = fopen("InertialData.csv","a");
		if (file_inertial == NULL)
		{
			ROS_INFO("Failed to open file");
			return;
		}
		fprintf(file_inertial,"DeltaAngleX,DeltaAngleY,DeltaAngleZ,DeltaVelocityX,DeltaVelocityY,DeltaVelocityZ");
		file_inertial_open = true;
	}
	fprintf(file_inertial,"\n%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", dt_x, dt_y, dt_z, dv_x, dv_y, dv_z);
	return;
}

void Msg01Callback(const hg_node::Msg_01::ConstPtr& msg);
void Msg04Callback(const hg_node::Msg_04::ConstPtr& msg);
void Msg0CCallback(const hg_node::Msg_0C::ConstPtr& msg);
void MsgA1Callback(const hg_node::Msg_A1::ConstPtr& msg);
void MsgACCallback(const hg_node::Msg_AC::ConstPtr& msg);
void MsgCACallback(const hg_node::Msg_CA::ConstPtr& msg);
//Intereaved Messages
void Msg02Callback(const hg_node::Msg_02::ConstPtr& msg);
void Msg05Callback(const hg_node::Msg_05::ConstPtr& msg);
void Msg0DCallback(const hg_node::Msg_0D::ConstPtr& msg);
void MsgA2Callback(const hg_node::Msg_A2::ConstPtr& msg);
void MsgADCallback(const hg_node::Msg_AD::ConstPtr& msg);
//Inertial Only
void MsgA3Callback(const hg_node::Msg_A3::ConstPtr& msg);
void MsgA9Callback(const hg_node::Msg_A9::ConstPtr& msg);
void MsgAECallback(const hg_node::Msg_AE::ConstPtr& msg);

int main(int argc, char *argv[])
{

	//init ROS variables
	ros::init(argc,argv, "ImuDataListener");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(1800);

	//find the correct topic name
	ros::master::V_TopicInfo master_topics;

	ros::Subscriber genericSub[32];


	// look for topics with 10s timeout.
	int timeout = 1000;
	int index = 0;
	do
	{	
		ros::Duration(0.01).sleep();
		//get list of topics
		ros::master::getTopics(master_topics); 
		//cycle through topics to find correct
		for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it!=master_topics.end();it++)
		{
			const ros::master::TopicInfo& info = *it;
			if (info.name.find("Output/0x") != std::string::npos) // start listening only to relevant topics
			{
				//HG4930
				if (info.name.find("0x01") != std::string::npos) // start listening only to relevant topics				
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg01Callback);
				if (info.name.find("0x02") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg02Callback);
				if (info.name.find("0xCA") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgCACallback);
				//HG1120
				if (info.name.find("0x04") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg04Callback);
				if (info.name.find("0x05") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg05Callback);
				if (info.name.find("0x0C") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg0CCallback);
				if (info.name.find("0x0D") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,Msg0DCallback);
				//HGuide i300
				if (info.name.find("0xA1") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgA1Callback);
				if (info.name.find("0xA2") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgA2Callback);
				if (info.name.find("0xAC") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgACCallback);
				if (info.name.find("0xAD") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgADCallback);
				if (info.name.find("0xA3") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgA3Callback);
				if (info.name.find("0xA9") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgA9Callback);
				if (info.name.find("0xAE") != std::string::npos) // start listening only to relevant topics
					genericSub[index++] = n.subscribe(info.name.c_str(),5,MsgAECallback);

				ROS_INFO("Listening to %s topic",info.name.c_str());
			}
				
		}
	timeout--;
	}while(index == 0 && timeout);
	

	ROS_INFO("Saving to csv...");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	fclose(file_ctrl);
	fclose(file_inertial);
	
	return 0;
}



//Message callbacks
void Msg01Callback(const hg_node::Msg_01::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
void Msg04Callback(const hg_node::Msg_04::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
void Msg0CCallback(const hg_node::Msg_0C::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
void MsgA1Callback(const hg_node::Msg_A1::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
void MsgACCallback(const hg_node::Msg_AC::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
void MsgCACallback(const hg_node::Msg_CA::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	return;
}
//Intereaved Messages
void Msg02Callback(const hg_node::Msg_02::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void Msg05Callback(const hg_node::Msg_05::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void Msg0DCallback(const hg_node::Msg_0D::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void MsgA2Callback(const hg_node::Msg_A2::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void MsgADCallback(const hg_node::Msg_AD::ConstPtr& msg)
{
	printControl(msg->AngularRateX,msg->AngularRateY,msg->AngularRateZ,msg->LinearAccelerationX,msg->LinearAccelerationY,msg->LinearAccelerationZ);
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
//Inertial Only
void MsgA3Callback(const hg_node::Msg_A3::ConstPtr& msg)
{
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void MsgA9Callback(const hg_node::Msg_A9::ConstPtr& msg)
{
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}
void MsgAECallback(const hg_node::Msg_AE::ConstPtr& msg)
{
	printInertial(msg->DeltaAngleX,msg->DeltaAngleY,msg->DeltaAngleZ,msg->DeltaVelocityX,msg->DeltaVelocityY,msg->DeltaVelocityZ);
	return;
}

