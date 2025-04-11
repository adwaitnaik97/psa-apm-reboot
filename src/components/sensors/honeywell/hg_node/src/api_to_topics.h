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

All technology that leaves the United States is subject to export regulations. This manual contains technology that has an Export Commodity Classification of EAR99.
This technology generally will not require a license to be exported or reexported. 
However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern, or in support of a prohibited end-use, you may be required to obtain a license.
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

#include <sstream>

#ifndef HG_UTILS
	#define PI 3.1415926535897932
	#define RAD_TO_DEG 180/PI
	#define DEG_TO_RAD PI/180
	#define HG_UTILS
#endif

//unit types
#define UNIT_HG4930 0
#define UNIT_HG1120 1
#define UNIT_I300   2

void initPubs(ros::NodeHandle * n);
void initSubs(ros::NodeHandle * n);
void stopSubs(void);
void stopPubs(void);

//utility function to publish standard IMU message
void getImuData(uint8_t * buffer, uint32_t bufferSize, uint8_t msgID);

/*---------------------------------------------------------------------------------------------------------------------------*/
//ROS Messages Callbacks
/*---------------------------------------------------------------------------------------------------------------------------*/
//Imu Message
void PubImu(float rateX,float rateY,float rateZ,float accelX,float accelY,float accelZ);
//Define covariance for the std imu message
void setImuCovariance(uint8_t unit_type);


