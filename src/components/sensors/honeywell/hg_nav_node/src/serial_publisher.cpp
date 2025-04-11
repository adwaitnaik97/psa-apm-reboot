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

//Termios lib
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
int getBaudRate(int baudrate);

//Custom messages
#include "api_to_topics.h"

//HGuide API
#include "include/HGuideAPI.h"

#define BUFFER_SIZE 4096 //4kB
#define BAUDRATE B921600	// 921.6 kBaud

uint8_t * RxBuffer;
uint8_t * TransmitBuffer;
int fh;

//messages for transformation
Msg_6403 GeoPosition;
Msg_6405 EulerAttitudes;
Msg_2311 InertialData;
Msg_6504 NEDVelocity;
Msg_2001 InstallData;

int main(int argc, char *argv[])
{
	fh = 0;

	//Init ROS variables
	ros::init(argc,argv, "HGuide");

	ros::NodeHandle n;


	int count = 0;
	
	//Open USB Serial Connection	
	struct termios serial;
	if (argc <= 1)
	{
		ROS_INFO("Using default COM: \"/dev/ttyUSB0\"");
		argv[1] = (char*)"/dev/ttyUSB0";
	}


	fh = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);

	if(fh == -1)
	{
		ROS_ERROR("Open \"%s\" failed",argv[1]);
		stopSubs();
		stopPubs();
		return -1;
	}
	ROS_INFO("Open \"%s\" success", argv[1]);


	//Serial Configuration
	serial.c_iflag = 0;
	serial.c_oflag = 0;
	serial.c_lflag = 0;
	serial.c_cflag = 0;
	
	serial.c_cc[VMIN] = 0;
	serial.c_cc[VTIME] = 0;

	if (argc >= 3)
	{
		int baudrate = atoi(argv[2]);
		if (getBaudRate(baudrate)==-1)
		{	
			ROS_WARN("Invalid Baudrate Input! - %d\nUSING PREVIOUSLY SET BAUDRATE",baudrate);
		}
		else
		{
			ROS_INFO("Using %d Baudrate",baudrate);
			serial.c_cflag = getBaudRate(baudrate)|CS8 | CREAD;
		}	

	}	
	else
	{
		ROS_INFO("Using default Baudrate: 921600");
		serial.c_cflag = BAUDRATE | CS8 | CREAD;
	}
	

	if (tcsetattr(fh, TCSANOW, &serial)<0)
	{
		ROS_ERROR("Error Setting configuration");
		return -3;
	}
	
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
	initSubs(&n);
	
	int pointer;
	int readCount = 0;
	int endOffset = 0;
	int status = 2;

	ros::Rate loop_rate(200); // 200 Hz

	while (ros::ok())
	{
		//Copy the remainder of previous reading before the newone
		memcpy(RxBuffer, RxBufferAct, (RxBufferEnd - RxBufferAct));
		//Reset the end of the buffer
		RxBufferEnd = RxBuffer + (RxBufferEnd - RxBufferAct);
		//Set Reading pointer to beginning
		RxBufferAct = RxBuffer;
		
		readCount = read(fh, RxBufferEnd, BUFFER_SIZE);

		endOffset = 0;
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
							processMessage(RxBufferAct,*(uint32_t*)(RxBufferAct),MessageId);
							
							//Process cases for STD messages and transformations
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
		
		ros::spinOnce();
		if(!loop_rate.sleep())
			ROS_WARN("CYCLE TIME NOT MET!");
	}//WHILE ros::ok()
	
	stopSubs();
	stopPubs();
	free(RxBuffer);
	free(TransmitBuffer);
	close(fh);
	
	return 0;
}


int getBaudRate(int baudrate)
{
	switch (baudrate)
	{
		case 50: return B50;
		case 75: return B75;
		case 110: return B110;
		case 134: return B134;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
		case 1800: return B1800;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		case 460800: return B460800;
		case 500000: return B500000;
		case 576000: return B576000;
		case 921600: return B921600;
		case 1000000: return B1000000;
		case 1152000: return B1152000;
		case 1500000: return B1500000;
		case 2000000: return B2000000;
		case 2500000: return B2500000;
		case 3000000: return B3000000;
		case 3500000: return B3500000;
		case 4000000: return B4000000;
		default:return -1; 
	}
}
