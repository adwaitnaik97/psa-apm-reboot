#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_2001.h>
hg_nav_node::Msg_2001 msgStruct_2001;

bool Msg_2001_pub_initialized = false;

ros::Publisher Msg_2001_pub;
void init_2001(ros::NodeHandle * n){
	Msg_2001_pub = n->advertise<hg_nav_node::Msg_2001>(MSG_2001_PATH, 5);
	Msg_2001_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_2001_PATH);
	return;
}

void stop_2001(void){
	Msg_2001_pub.shutdown();
	Msg_2001_pub_initialized = false;
	ROS_INFO("0x2001 stopped");
	return;
}

// Msg_2001 to Topic
void convert(Msg_2001 messageIn, hg_nav_node::Msg_2001 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->IMUSerialNumber[index] = messageIn.IMUSerialNumber[index];
	}

	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->ISA_Performance_Grade[index] = messageIn.ISA_Performance_Grade[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSoftwareVersion[index] = messageIn.IMUSoftwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FPGA_VERSION[index] = messageIn.FPGA_VERSION[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FSBL_VERSION[index] = messageIn.FSBL_VERSION[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SSBL_VERSION[index] = messageIn.SSBL_VERSION[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersion[index] = messageIn.HGuideSoftwareVersion[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersionBuildDate[index] = messageIn.HGuideSoftwareVersionBuildDate[index];
	}

	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersionBuildTime[index] = messageIn.HGuideSoftwareVersionBuildTime[index];
	}

	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->INS_CONFIG_FILEID[index] = messageIn.INS_CONFIG_FILEID[index];
	}

	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->SSBL_REGINIT_FILEID[index] = messageIn.SSBL_REGINIT_FILEID[index];
	}

	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->SSBL_FLASHLDR_FILEID[index] = messageIn.SSBL_FLASHLDR_FILEID[index];
	}

	for (unsigned int index = 0; index < 10; index++)
	{
		messageOut->HGuideSerialNumber[index] = messageIn.HGuideSerialNumber[index];
	}

	for (unsigned int index = 0; index < 10; index++)
	{
		messageOut->HGuidePartNumber[index] = messageIn.HGuidePartNumber[index];
	}

	messageOut->device_configuration = messageIn.device_configuration;
	messageOut->CaseToNavLeverArmX = messageIn.CaseToNavLeverArmX;
	messageOut->CaseToNavLeverArmY = messageIn.CaseToNavLeverArmY;
	messageOut->CaseToNavLeverArmZ = messageIn.CaseToNavLeverArmZ;
	messageOut->CaseToNavQuaternionS = messageIn.CaseToNavQuaternionS;
	messageOut->CaseToNavQuaternionI = messageIn.CaseToNavQuaternionI;
	messageOut->CaseToNavQuaternionJ = messageIn.CaseToNavQuaternionJ;
	messageOut->CaseToNavQuaternionK = messageIn.CaseToNavQuaternionK;
	messageOut->VehicleLeverArmX = messageIn.VehicleLeverArmX;
	messageOut->VehicleLeverArmY = messageIn.VehicleLeverArmY;
	messageOut->VehicleLeverArmZ = messageIn.VehicleLeverArmZ;
	messageOut->VehicleQuaterionS = messageIn.VehicleQuaterionS;
	messageOut->VehicleQuaterionI = messageIn.VehicleQuaterionI;
	messageOut->VehicleQuaterionJ = messageIn.VehicleQuaterionJ;
	messageOut->VehicleQuaterionK = messageIn.VehicleQuaterionK;
	messageOut->RF1AntennaLeverArmRSS = messageIn.RF1AntennaLeverArmRSS;
	messageOut->RF1AntennaLeverArmX = messageIn.RF1AntennaLeverArmX;
	messageOut->RF1AntennaLeverArmY = messageIn.RF1AntennaLeverArmY;
	messageOut->RF1AntennaLeverArmZ = messageIn.RF1AntennaLeverArmZ;
	messageOut->HGNSI_AntennaLeverArmX = messageIn.HGNSI_AntennaLeverArmX;
	messageOut->HGNSI_AntennaLeverArmY = messageIn.HGNSI_AntennaLeverArmY;
	messageOut->HGNSI_AntennaLeverArmZ = messageIn.HGNSI_AntennaLeverArmZ;
	messageOut->RF2AntennaLeverArmX = messageIn.RF2AntennaLeverArmX;
	messageOut->RF2AntennaLeverArmY = messageIn.RF2AntennaLeverArmY;
	messageOut->RF2AntennaLeverArmZ = messageIn.RF2AntennaLeverArmZ;
	messageOut->GPSantToCaseQuaternionS = messageIn.GPSantToCaseQuaternionS;
	messageOut->GPSantToCaseQuaternionI = messageIn.GPSantToCaseQuaternionI;
	messageOut->GPSantToCaseQuaternionJ = messageIn.GPSantToCaseQuaternionJ;
	messageOut->GPSantToCaseQuaternionK = messageIn.GPSantToCaseQuaternionK;
}

// Topic to Msg_2001
void convert(hg_nav_node::Msg_2001 messageIn, Msg_2001 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->IMUSerialNumber[index] = messageIn.IMUSerialNumber[index];
	}
	for (unsigned int index = 0; index < 8; index++)
	{
		messageOut->ISA_Performance_Grade[index] = messageIn.ISA_Performance_Grade[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->IMUSoftwareVersion[index] = messageIn.IMUSoftwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FPGA_VERSION[index] = messageIn.FPGA_VERSION[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->FSBL_VERSION[index] = messageIn.FSBL_VERSION[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->SSBL_VERSION[index] = messageIn.SSBL_VERSION[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersion[index] = messageIn.HGuideSoftwareVersion[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersionBuildDate[index] = messageIn.HGuideSoftwareVersionBuildDate[index];
	}
	for (unsigned int index = 0; index < 16; index++)
	{
		messageOut->HGuideSoftwareVersionBuildTime[index] = messageIn.HGuideSoftwareVersionBuildTime[index];
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->INS_CONFIG_FILEID[index] = messageIn.INS_CONFIG_FILEID[index];
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->SSBL_REGINIT_FILEID[index] = messageIn.SSBL_REGINIT_FILEID[index];
	}
	for (unsigned int index = 0; index < 4; index++)
	{
		messageOut->SSBL_FLASHLDR_FILEID[index] = messageIn.SSBL_FLASHLDR_FILEID[index];
	}
	for (unsigned int index = 0; index < 10; index++)
	{
		messageOut->HGuideSerialNumber[index] = messageIn.HGuideSerialNumber[index];
	}
	for (unsigned int index = 0; index < 10; index++)
	{
		messageOut->HGuidePartNumber[index] = messageIn.HGuidePartNumber[index];
	}

	messageOut->device_configuration = messageIn.device_configuration;
	messageOut->CaseToNavLeverArmX = messageIn.CaseToNavLeverArmX;
	messageOut->CaseToNavLeverArmY = messageIn.CaseToNavLeverArmY;
	messageOut->CaseToNavLeverArmZ = messageIn.CaseToNavLeverArmZ;
	messageOut->CaseToNavQuaternionS = messageIn.CaseToNavQuaternionS;
	messageOut->CaseToNavQuaternionI = messageIn.CaseToNavQuaternionI;
	messageOut->CaseToNavQuaternionJ = messageIn.CaseToNavQuaternionJ;
	messageOut->CaseToNavQuaternionK = messageIn.CaseToNavQuaternionK;
	messageOut->VehicleLeverArmX = messageIn.VehicleLeverArmX;
	messageOut->VehicleLeverArmY = messageIn.VehicleLeverArmY;
	messageOut->VehicleLeverArmZ = messageIn.VehicleLeverArmZ;
	messageOut->VehicleQuaterionS = messageIn.VehicleQuaterionS;
	messageOut->VehicleQuaterionI = messageIn.VehicleQuaterionI;
	messageOut->VehicleQuaterionJ = messageIn.VehicleQuaterionJ;
	messageOut->VehicleQuaterionK = messageIn.VehicleQuaterionK;
	messageOut->RF1AntennaLeverArmRSS = messageIn.RF1AntennaLeverArmRSS;
	messageOut->RF1AntennaLeverArmX = messageIn.RF1AntennaLeverArmX;
	messageOut->RF1AntennaLeverArmY = messageIn.RF1AntennaLeverArmY;
	messageOut->RF1AntennaLeverArmZ = messageIn.RF1AntennaLeverArmZ;
	messageOut->HGNSI_AntennaLeverArmX = messageIn.HGNSI_AntennaLeverArmX;
	messageOut->HGNSI_AntennaLeverArmY = messageIn.HGNSI_AntennaLeverArmY;
	messageOut->HGNSI_AntennaLeverArmZ = messageIn.HGNSI_AntennaLeverArmZ;
	messageOut->RF2AntennaLeverArmX = messageIn.RF2AntennaLeverArmX;
	messageOut->RF2AntennaLeverArmY = messageIn.RF2AntennaLeverArmY;
	messageOut->RF2AntennaLeverArmZ = messageIn.RF2AntennaLeverArmZ;
	messageOut->GPSantToCaseQuaternionS = messageIn.GPSantToCaseQuaternionS;
	messageOut->GPSantToCaseQuaternionI = messageIn.GPSantToCaseQuaternionI;
	messageOut->GPSantToCaseQuaternionJ = messageIn.GPSantToCaseQuaternionJ;
	messageOut->GPSantToCaseQuaternionK = messageIn.GPSantToCaseQuaternionK;
}

void Msg_2001_pub_callback(uint8_t * buffer)
{
	Msg_2001 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x2001 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_2001);
	ROS_DEBUG("Message 0x2001 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_2001_pub_initialized == false){
		init_2001(getRosHandle());}
	// Publish the message
	Msg_2001_pub.publish(msgStruct_2001);
	return;
}
