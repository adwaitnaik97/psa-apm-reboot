/*
	Simple example to read and save data from HG navigator Data publisher
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

#include "api_to_topics.h"

#include <sstream>
#include <fstream>

#define LINE_ARRAY_SIZE 1024

using namespace std;

//pointer to csv file
ofstream f6403;
bool f6403_first=true;




//Geodetic Position Message Callback
void hgGeoCallback(const hg_nav_node::Msg_6403::ConstPtr& Message)
{
	//Create message structure
	Msg_6403 messageIn;
	//Allocate Print buffer
	char lineChar[LINE_ARRAY_SIZE] = { 0 };
	
	//Convert Topic to Message
	convert(*Message,&messageIn);
	
	//Create file for data saving and print header
	if (f6403_first)
	{
		f6403.open("HgInsData-6403.csv");
		f6403<<"SystemTov,GpsTov,INSMode,GPSMode,Latitude,Longitude,Altitude,LatSTDV,LonSTDV,AltSTDV";
		f6403_first = false;
	}
	f6403 << "\n" << messageIn.systemTov<<","<<messageIn.gpsTov<<","<<\
	messageIn.InsGnssSummary.INSMode<<","<<messageIn.InsGnssSummary.GPSMode<<","<<\
	messageIn.Latitude<<","<<messageIn.Longitude<<","<<messageIn.AltitudeHeightAboveEllipsoid<<","<<\
	messageIn.LatitudeSTDV<<","<<messageIn.LongitudeSTDV<<","<<messageIn.AltitudeHeightAboveEllipsoidSTDV;

	return;
}

int main(int argc, char *argv[])
{

	//init ROS variables
	ros::init(argc,argv, "SaveInsDataToCsv");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(200); // double the maximum output frequency

	
	
	//init Subscriber nodes
	ros::Subscriber HgGeoPosOutput_sub = n.subscribe(MSG_6403_PATH, 5,hgGeoCallback);
//not used subscribers	
//	ros::Subscriber HgNavOutput_sub = n.subscribe(MSG_2401_PATH, 5,hgNavOutputCallback);	
//	ros::Subscriber HgInertialOutput_sub = n.subscribe(MSG_2311_PATH, 5,hgInertialCallback);
//	ros::Subscriber HgEulerOutput_sub = n.subscribe(MSG_6405_PATH, 5,hgEulerCallback);

	




	ROS_INFO("Saving to csv...");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Close the Files
	if(!f6403_first) f6403.close();

	return 0;
}
