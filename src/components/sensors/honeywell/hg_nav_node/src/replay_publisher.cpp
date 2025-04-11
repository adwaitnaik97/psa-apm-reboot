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

#include <stdio.h>
#include <stdlib.h>
	
#include <fstream>
#include <iostream>
using namespace std;

//Custom messages
#include "api_to_topics.h"

//HGuide API
#include "include/HGuideAPI.h"

#define BUFFER_SIZE 4096 //4kB

uint8_t * RxBuffer;
uint8_t * TransmitBuffer;
int fh;

//messages for transformation
Msg_6403 GeoPosition;
Msg_6405 EulerAttitudes;
Msg_2311 InertialData;
Msg_6504 NEDVelocity;
Msg_2001 InstallData;

int fopen_s(FILE **f, const char*name,const char *mode)
{
	*f = fopen(name,mode);
	if (!*f)
		return-1;
	return 0;
}

size_t fread_s(void * ptr, size_t inBufferSize, size_t size, size_t count, FILE* stream)
{
	return fread(ptr,size,count,stream);
}

int main(int argc, char *argv[])
{
	fh = 0;

	//Init ROS variables
	ros::init(argc,argv, "HGuide");

	ros::NodeHandle n;


	int count = 0;
	
		//Check input arguments
	if (argc < 2)
	{
		printf("Usage:\nrosrun hg_nav_node replay_publisher_nav [SourceFileName]\n"
			"\t[SourceFileName]\t\tPath to the binary source file\n"
			"\t[Init Lat]\t\t(optional) Local vertical Latitude (rad)\n"
			"\t[Init Lon]\t\t(optional) Local vertical Longitude (rad)\n"
			"\t[Init Alt]\t\t(optional) Local vertical Altitude (m)\n"
			"\t[Init Heading]\t\t(optional) Local Vertical Heading (rad)\n");
		return -1;
	}

	//Read File definition
	char* SourceFileName = (char *)malloc(sizeof(char) * 256);
	FILE *s;

	/*----------------------------------------------------------------------------*/
	/*----------READ-INPUT-ARGUMENTS----------------------------------------------*/
	/*----------------------------------------------------------------------------*/

	//First Argument Source file
	if (argc >= 2)
	{
		strncpy(SourceFileName, argv[1], 256);
	}

	fopen_s(&s, SourceFileName, "rb");
	if (s == NULL)
	{
		printf("Failed to open source file!\n");
		return -2;
	}

	if (argc >= 6) // Custom LocalVertical Position
	{
		initLat     = atof(argv[2]);
		initLon     = atof(argv[3]);
		initAlt     = atof(argv[4]);
		initHeading = atof(argv[5]);
		
	}
	printf("Source File opened successfully\n");

	//Measure file size to calculate percentages
	fseek(s, 0, SEEK_END);
	long FileSize = ftell(s);
	fseek(s, 0, SEEK_SET);
	
	//Allocate read buffer
	int bytesRead = 0;
	int byteOffset = 0;
	int endOffset = 0;
	int status = 2;

	
	//Allocate memory for buffers
	RxBuffer = (uint8_t *)malloc(BUFFER_SIZE*2);
	uint8_t *RxBufferAct = RxBuffer;
	uint8_t *RxBufferEnd = RxBuffer;
	TransmitBuffer = (uint8_t *)malloc(BUFFER_SIZE);

	//pass variables to generic HGuide messages
	setVariables(BUFFER_SIZE, TransmitBuffer, fh, &n);

	ROS_INFO("STARTING BROADCAST\n\
	Published Topics:\n\
	\t\"/HGuide/Std/Imu\"\n\
	\t\"/HGuide/Std/Odometry\"\n\
	\t\"/HGuide/Std/NavSatFix\"\n\
	\t\"/HGuide/Std/TimeReference\"\n\
	\t\"/HGuide/Std/TwistWithCovarianceStamped\"");	
	
	//Init all publishers
	initPubs(&n);

	// Subscribe to Input messages
	// Callback functions will create the uint8_t message and send it over serial port (handle fh)
	//initSubs(&n);
	
	int pointer;
	int readCount = 0;
	int Offset = 0;

	ros::Rate loop_rate(200); // 200 Hz



	//Timing 
	double curTime = ros::Time::now().toSec();
	while (ros::ok())
	{
	
		Offset = -1*(RxBufferEnd - RxBufferAct);
		fseek(s, Offset , SEEK_CUR);
		readCount = (int) fread_s(RxBuffer, sizeof(RxBuffer), sizeof(uint8_t), BUFFER_SIZE, s);
		
		RxBufferAct = RxBuffer;
		RxBufferEnd = RxBuffer;
		
		if (readCount>0)
		{
			// Move the pointer to the end of the Read buffer
			RxBufferEnd += readCount;
			while (RxBufferAct<RxBufferEnd)
			{
				if (*(uint32_t*)(RxBufferAct) == 0xA5C381FF)
				{
					uint32_t MessageId = *(uint32_t*)(RxBufferAct + 4);
					uint32_t MessageLength = *(uint32_t*)(RxBufferAct + 8);
					
					//Does the length make sense?
					if (MessageLength > 0 && MessageLength < BUFFER_SIZE/4)
					{
						//Is there enough data in buffer to process the message?
						if (RxBufferAct <= RxBufferEnd-(MessageLength*4) && ((MessageLength*4)<BUFFER_SIZE))
						{
						
							//Wait for correct time
							if (MessageId == 0x2311) //Start of frame
							{	
								while (curTime+0.01>ros::Time::now().toSec())
								{
									usleep(2*1000);
								}
								curTime = ros::Time::now().toSec();
							}
							
							processMessage(RxBufferAct,*(uint32_t*)(RxBufferAct),MessageId);
							
							//Process cases for transformations
							switch (MessageId)
							{
								case 0x2001: 
									if (InstallData.IMUSerialNumber[0]==0)
									{
										ROS_INFO("Updating IMU installation attitude from 0x2001 Message");
										InstallData.Deserialize(RxBufferAct,RxBufferEnd-RxBufferAct);
										api_set_Installation(InstallData);
									}
									break;
								case 0x2311: InertialData.Deserialize(RxBufferAct,RxBufferEnd-RxBufferAct); break;
								case 0x6403: GeoPosition.Deserialize(RxBufferAct,RxBufferEnd-RxBufferAct);
									if (GeoPosition.InsGnssSummary.INSMode == 4 && initLat == -999.0)
									{
										initLat = GeoPosition.Latitude;
										initLon = GeoPosition.Longitude;
										initAlt = GeoPosition.AltitudeHeightAboveEllipsoid;
									}
									break;
								case 0x6405: EulerAttitudes.Deserialize(RxBufferAct,RxBufferEnd-RxBufferAct); break;
								case 0x6504: // Always Comes last
									NEDVelocity.Deserialize(RxBufferAct,RxBufferEnd-RxBufferAct); 
									processStdTopics(GeoPosition, EulerAttitudes, NEDVelocity, InertialData);
									break;
							}
						}
						else //get more data and process again
						{
							break;
						}
					}

				} // if *RxBufferAct == 0xA5C381FF
				//Move the read buffer pointer
				RxBufferAct++;
			} //while RxAct < RxEnd
		}
		else // End of file
		{
			ROS_INFO("End of file reached. Exitting...");
			break;
		}
		
	
		ros::spinOnce();
	}//WHILE ros::ok()
	
	//stopSubs();
	stopPubs();
	free(RxBuffer);
	free(TransmitBuffer);
	close(fh);
	
	return 0;
}




