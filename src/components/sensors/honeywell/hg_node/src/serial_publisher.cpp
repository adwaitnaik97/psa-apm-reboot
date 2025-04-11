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

All technology that leaves the United States is subject to export regulations. This manual contains technology that has an Export Commodity Classification of EAR99.
This technology generally will not require a license to be exported or reexported. 
However, if you plan to export this item to an embargoed or sanctioned country, to a party of concern, or in support of a prohibited end-use, you may be required to obtain a license.
*/

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

//Termios lib
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int getBaudRate(int baudrate);
int getMessageLen(uint32_t messageID);

//Custom messages
#include "api_to_topics.h"

//HGuide API
#include "include/HGuideAPI.h"

#include <sstream>

#define BUF_SIZE 4096
#define BAUDRATE B1000000	// 1 MBaud
#define SYNC_BYTE 0x0E


int main(int argc, char *argv[])
{
	int fh;
	int status = 0;
	
	//Init ROS variables
	ros::init(argc,argv, "HGuide");

	ros::NodeHandle n;

	int count = 0;
	
	//Open USB Serial Connection	
	struct termios serial;
	if (argc <= 1)
	{
		ROS_INFO("Using default COM: \"/dev/ttyUSB0\"\n");
		argv[1] = (char*)"/dev/ttyUSB0";
	}


	fh = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);

	if(fh == -1)
	{
		ROS_ERROR("Open %s failed\n",argv[1]);
		return -1;
	}
	ROS_INFO("Open %s success\n", argv[1]);


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
		ROS_INFO("Using default Baudrate: 1000000");
		serial.c_cflag = BAUDRATE | CS8 | CREAD;
	}

	if (tcsetattr(fh, TCSANOW, &serial)<0)
	{
		ROS_ERROR("Error Setting configuration\n");
		return -3;
	}
	
	//Define the RX Buffer
	uint8_t *RxBuffer;
	RxBuffer = (uint8_t*)malloc(BUF_SIZE*2);
	uint8_t *RxBufferAct = RxBuffer;
	uint8_t *RxBufferEnd = RxBuffer;

	ROS_INFO("STARTING BROADCAST\n");

	int readCount=0;
	
	
	setVariables(BUF_SIZE, 0, 0, &n);
	
	//Start publishers
	initPubs(&n);
	
	//TBD: get IMU output frequency from messages
	double frequency = 600;
	ros::Rate loop_rate(frequency);
	
	while (ros::ok())
	{
		//Copy the remainder of previous reading before the newone
		memcpy(RxBuffer, RxBufferAct, (RxBufferEnd - RxBufferAct));
		//Reset the end of the buffer
		RxBufferEnd = RxBuffer + (RxBufferEnd - RxBufferAct);
		//Set Reading pointer to beginning
		RxBufferAct = RxBuffer;
		
		readCount = read(fh, RxBufferEnd, BUF_SIZE);

		if (readCount>0)
		{
			// Move the pointer to the end of the Read buffer
			RxBufferEnd += readCount;

			while (RxBufferAct<=RxBufferEnd-1)
			{
				if (*(uint8_t*)RxBufferAct == SYNC_BYTE)
				{
					uint32_t messageID = *(uint8_t*)(RxBufferAct+1);
					if (RxBufferEnd-RxBufferAct >= getMessageLen(messageID))
					{
						//Publish sensor_msgs::imu message
						getImuData(RxBufferAct,RxBufferEnd-RxBufferAct,messageID);
						
						if(processMessage(RxBufferAct,*(uint8_t*)RxBufferAct,messageID))
						{
							RxBufferAct += getMessageLen(messageID) - 1;
						}
					}
					else //Get more data to process the message
					{
						break;
					}

				} // if *RxBufferAct == SYNC_BYTE
				//Move the read buffer pointer
				RxBufferAct++;
			} //while RxAct < RxEnd
			
		}
		ros::spinOnce();
		if(!loop_rate.sleep())
			ROS_WARN("CYCLE TIME NOT MET!");
	}//WHILE ros::ok()
	free(RxBuffer);
	close(fh);
	
	return 0;
}

int getMessageLen(uint32_t messageID)
{
	switch(messageID)
	{
		case 0xCA:
		case 0xA1:
		case 0x01: return 20;
		case 0xA2:
		case 0x02: return 44;
		case 0xAC:
		case 0x0C:
		case 0x04: return 26;
		case 0xAD:
		case 0x0D:
		case 0xA5: return 50;
		case 0xA3: return 32;
		case 0xA9: return 56;
		case 0xAE: return 38;
		case 0xB0: return 82;
		case 0xB1: return 30;
		case 0xB2: return 42;
		case 0xB3: return 24;
		case 0xB4: return 26;
		case 0xF0:
		case 0xF1:
		case 0xF2:
		case 0xF3:
		case 0xF4:
		case 0xF5:
		case 0xF6:
		case 0xF7:
		case 0xF8:
		case 0xF9:
		case 0xFA:
		case 0xFB:
		case 0xFC:
		case 0xFD:
		case 0xFF: return 12;
		default: return 82;
		
	}
	return 128;
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



