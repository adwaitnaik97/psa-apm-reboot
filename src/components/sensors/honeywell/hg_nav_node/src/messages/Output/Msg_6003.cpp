#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6003.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6003 msgStruct_6003;

bool Msg_6003_pub_initialized = false;

ros::Publisher Msg_6003_pub;
void init_6003(ros::NodeHandle * n){
	Msg_6003_pub = n->advertise<hg_nav_node::Msg_6003>(MSG_6003_PATH, 5);
	Msg_6003_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6003_PATH);
	return;
}

void stop_6003(void){
	Msg_6003_pub.shutdown();
	Msg_6003_pub_initialized = false;
	ROS_INFO("0x6003 stopped");
	return;
}

// Msg_6003 to Topic
void convert(Msg_6003 messageIn, hg_nav_node::Msg_6003 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;

	messageOut->InsGnssSummary.INSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.INSMode);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.GPSMode);
	messageOut->ImuLeverArmsX = messageIn.ImuLeverArmsX;
	messageOut->ImuLeverArmsY = messageIn.ImuLeverArmsY;
	messageOut->ImuLeverArmsZ = messageIn.ImuLeverArmsZ;
	messageOut->VehicleEulerAnglesRoll = messageIn.VehicleEulerAnglesRoll;
	messageOut->VehicleEulerAnglesPitch = messageIn.VehicleEulerAnglesPitch;
	messageOut->VehicleEulerAnglesTrueHeading = messageIn.VehicleEulerAnglesTrueHeading;
	messageOut->CaseToNavQuaterionS = messageIn.CaseToNavQuaterionS;
	messageOut->CaseToNavQuaterionI = messageIn.CaseToNavQuaterionI;
	messageOut->CaseToNavQuaterionJ = messageIn.CaseToNavQuaterionJ;
	messageOut->CaseToNavQuaterionK = messageIn.CaseToNavQuaterionK;
	messageOut->RF1AntennaLeverArmStdv = messageIn.RF1AntennaLeverArmStdv;
	messageOut->RF1AntennaLeverArmX = messageIn.RF1AntennaLeverArmX;
	messageOut->RF1AntennaLeverArmY = messageIn.RF1AntennaLeverArmY;
	messageOut->RF1AntennaLeverArmZ = messageIn.RF1AntennaLeverArmZ;
	messageOut->RF2AuxiliaryAntennaLeverArmX = messageIn.RF2AuxiliaryAntennaLeverArmX;
	messageOut->RF2AuxiliaryAntennaLeverArmY = messageIn.RF2AuxiliaryAntennaLeverArmY;
	messageOut->RF2AuxiliaryAntennaLeverArmZ = messageIn.RF2AuxiliaryAntennaLeverArmZ;
	messageOut->RF12LOSPitchEst = messageIn.RF12LOSPitchEst;
	messageOut->RF12LOSTrueHeadingEst = messageIn.RF12LOSTrueHeadingEst;
	messageOut->AntennaBoresightPitchEstStdv = messageIn.AntennaBoresightPitchEstStdv;
	messageOut->AntennaBoresightTrueHeadingEstStdv = messageIn.AntennaBoresightTrueHeadingEstStdv;
	messageOut->RF1MainAntennaLeverArmEstX = messageIn.RF1MainAntennaLeverArmEstX;
	messageOut->RF1MainAntennaLeverArmEstY = messageIn.RF1MainAntennaLeverArmEstY;
	messageOut->RF1MainAntennaLeverArmEstZ = messageIn.RF1MainAntennaLeverArmEstZ;
	messageOut->RF1MainAntennaLeverArmEstStdvX = messageIn.RF1MainAntennaLeverArmEstStdvX;
	messageOut->RF1MainAntennaLeverArmEstStdvY = messageIn.RF1MainAntennaLeverArmEstStdvY;
	messageOut->RF1MainAntennaLeverArmEstStdvZ = messageIn.RF1MainAntennaLeverArmEstStdvZ;
	messageOut->OdometerLeverArmEstX = messageIn.OdometerLeverArmEstX;
	messageOut->OdometerLeverArmEstY = messageIn.OdometerLeverArmEstY;
	messageOut->OdometerLeverArmEstZ = messageIn.OdometerLeverArmEstZ;
	messageOut->OdometerLeverArmEstStdvX = messageIn.OdometerLeverArmEstStdvX;
	messageOut->OdometerLeverArmEstStdvY = messageIn.OdometerLeverArmEstStdvY;
	messageOut->OdometerLeverArmEstStdvZ = messageIn.OdometerLeverArmEstStdvZ;
	messageOut->OdometerBoresightTrueHeading = messageIn.OdometerBoresightTrueHeading;
	messageOut->OdometerBoresightPitch = messageIn.OdometerBoresightPitch;
	messageOut->OdometerSFStdv = messageIn.OdometerSFStdv;
	messageOut->OdometerLeverArmStoredX = messageIn.OdometerLeverArmStoredX;
	messageOut->OdometerLeverArmStoredY = messageIn.OdometerLeverArmStoredY;
	messageOut->OdometerLeverArmStoredZ = messageIn.OdometerLeverArmStoredZ;
	messageOut->RF12BoresightTrueHeadingAdjustment = messageIn.RF12BoresightTrueHeadingAdjustment;
	messageOut->RF12BoresightPitchAdjustment = messageIn.RF12BoresightPitchAdjustment;
}

// Topic to Msg_6003
void convert(hg_nav_node::Msg_6003 messageIn, Msg_6003 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->systemTov = messageIn.systemTov;
	messageOut->gpsTov = messageIn.gpsTov;

	messageOut->InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(messageIn.InsGnssSummary.INSMode.value);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(messageIn.InsGnssSummary.GPSMode.value);
	messageOut->ImuLeverArmsX = messageIn.ImuLeverArmsX;
	messageOut->ImuLeverArmsY = messageIn.ImuLeverArmsY;
	messageOut->ImuLeverArmsZ = messageIn.ImuLeverArmsZ;
	messageOut->VehicleEulerAnglesRoll = messageIn.VehicleEulerAnglesRoll;
	messageOut->VehicleEulerAnglesPitch = messageIn.VehicleEulerAnglesPitch;
	messageOut->VehicleEulerAnglesTrueHeading = messageIn.VehicleEulerAnglesTrueHeading;
	messageOut->CaseToNavQuaterionS = messageIn.CaseToNavQuaterionS;
	messageOut->CaseToNavQuaterionI = messageIn.CaseToNavQuaterionI;
	messageOut->CaseToNavQuaterionJ = messageIn.CaseToNavQuaterionJ;
	messageOut->CaseToNavQuaterionK = messageIn.CaseToNavQuaterionK;
	messageOut->RF1AntennaLeverArmStdv = messageIn.RF1AntennaLeverArmStdv;
	messageOut->RF1AntennaLeverArmX = messageIn.RF1AntennaLeverArmX;
	messageOut->RF1AntennaLeverArmY = messageIn.RF1AntennaLeverArmY;
	messageOut->RF1AntennaLeverArmZ = messageIn.RF1AntennaLeverArmZ;
	messageOut->RF2AuxiliaryAntennaLeverArmX = messageIn.RF2AuxiliaryAntennaLeverArmX;
	messageOut->RF2AuxiliaryAntennaLeverArmY = messageIn.RF2AuxiliaryAntennaLeverArmY;
	messageOut->RF2AuxiliaryAntennaLeverArmZ = messageIn.RF2AuxiliaryAntennaLeverArmZ;
	messageOut->RF12LOSPitchEst = messageIn.RF12LOSPitchEst;
	messageOut->RF12LOSTrueHeadingEst = messageIn.RF12LOSTrueHeadingEst;
	messageOut->AntennaBoresightPitchEstStdv = messageIn.AntennaBoresightPitchEstStdv;
	messageOut->AntennaBoresightTrueHeadingEstStdv = messageIn.AntennaBoresightTrueHeadingEstStdv;
	messageOut->RF1MainAntennaLeverArmEstX = messageIn.RF1MainAntennaLeverArmEstX;
	messageOut->RF1MainAntennaLeverArmEstY = messageIn.RF1MainAntennaLeverArmEstY;
	messageOut->RF1MainAntennaLeverArmEstZ = messageIn.RF1MainAntennaLeverArmEstZ;
	messageOut->RF1MainAntennaLeverArmEstStdvX = messageIn.RF1MainAntennaLeverArmEstStdvX;
	messageOut->RF1MainAntennaLeverArmEstStdvY = messageIn.RF1MainAntennaLeverArmEstStdvY;
	messageOut->RF1MainAntennaLeverArmEstStdvZ = messageIn.RF1MainAntennaLeverArmEstStdvZ;
	messageOut->OdometerLeverArmEstX = messageIn.OdometerLeverArmEstX;
	messageOut->OdometerLeverArmEstY = messageIn.OdometerLeverArmEstY;
	messageOut->OdometerLeverArmEstZ = messageIn.OdometerLeverArmEstZ;
	messageOut->OdometerLeverArmEstStdvX = messageIn.OdometerLeverArmEstStdvX;
	messageOut->OdometerLeverArmEstStdvY = messageIn.OdometerLeverArmEstStdvY;
	messageOut->OdometerLeverArmEstStdvZ = messageIn.OdometerLeverArmEstStdvZ;
	messageOut->OdometerBoresightTrueHeading = messageIn.OdometerBoresightTrueHeading;
	messageOut->OdometerBoresightPitch = messageIn.OdometerBoresightPitch;
	messageOut->OdometerSFStdv = messageIn.OdometerSFStdv;
	messageOut->OdometerLeverArmStoredX = messageIn.OdometerLeverArmStoredX;
	messageOut->OdometerLeverArmStoredY = messageIn.OdometerLeverArmStoredY;
	messageOut->OdometerLeverArmStoredZ = messageIn.OdometerLeverArmStoredZ;
	messageOut->RF12BoresightTrueHeadingAdjustment = messageIn.RF12BoresightTrueHeadingAdjustment;
	messageOut->RF12BoresightPitchAdjustment = messageIn.RF12BoresightPitchAdjustment;
}

void Msg_6003_pub_callback(uint8_t * buffer)
{
	Msg_6003 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6003 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6003);
	ROS_DEBUG("Message 0x6003 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6003_pub_initialized == false){
		init_6003(getRosHandle());}
	// Publish the message
	Msg_6003_pub.publish(msgStruct_6003);
	return;
}
