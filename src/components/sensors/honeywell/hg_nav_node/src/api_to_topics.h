/*
Library to convert API Message structures to ROS Topic messages
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

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"



//HGuide API
#include "include/HGuideAPI.h"

//Custom messages
#include "messages/message_support.h"

//standard Messages
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <sstream>

//Pose transformation
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
void geo_to_ecef(double lat, double lon, double alt,double * ecef1,double * ecef2,double * ecef3);
void lla2mtrdiff(double in1lat, double in1lon, double in1alt,double in2lat, double in2lon, double in2alt,double * outN, double * outE, double * outD);

#ifndef HG_UTILS
	#define PI 3.1415926535897932
	#define RAD_TO_DEG 180/PI
	#define DEG_TO_RAD PI/180
	#define HG_UTILS
#endif

double initLat = -999.0;
double initLon = -999.0;
double initAlt = -999.0;
double initHeading = 0;

void initPubs(ros::NodeHandle * n);
void initSubs(ros::NodeHandle * n);
void stopSubs(void);
void stopPubs(void);

//Set installation for propper Std/Imu msg rotation
void api_set_Installation(Msg_2001 input);

void processStdTopics(Msg_6403 GeoPosition, Msg_6405 EulerAttitudes, Msg_6504 NEDVelocity, Msg_2311 InertialData);
//Publish Transformation
geometry_msgs::TransformStamped WorldToBody;
geometry_msgs::TransformStamped bodyToLocalVertical;
geometry_msgs::TransformStamped ObsBody;
tf2::Quaternion q;


/*---------------------------------------------------------------------------------------------------------------------------*/
//ROS Messages Callbacks
/*---------------------------------------------------------------------------------------------------------------------------*/
//NavSatFix Message
void PubNavSatFix(Msg_6403 Message);

//Imu Message
void PubImu(Msg_2311 InertialOutputMsg, Msg_6405 EulerAttitudeOutputMsg);

//Odometry Message
void PubOdometry(tf2::Quaternion q, double x, double y, double z,Msg_6403 GeodeticPositionOutputMsg, Msg_6405 EulerAttitudeOutputMsg, Msg_6504 NEDVelocityOutputMsg);

//Twist With Covariance Stamped message
void PubTwistWithCovariance(Msg_6405 EulerAttitudeOutputMsg, Msg_6504 NEDVelocityOutputMsg);

//Time reference in UTC time
void PubTimeReference(double gpsTime);


