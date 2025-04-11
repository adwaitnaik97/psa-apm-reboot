#pragma once
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

//HGuide API
#include <include/HGuideAPI.h>

// print variables
static uint32_t bufSize=0;
static uint8_t * TxBuffer;
static int serialHandle=0;
static ros::NodeHandle * internalNodeHandle;
//Get functions
uint32_t getBufSize(void);
uint8_t * getTxBuffer(void);
int getSerialHandle(void);
ros::NodeHandle * getRosHandle(void);

//Set the variables needed for processing
void setVariables(uint32_t sizeOfBuffer, uint8_t * transmitBuffer, int sh, ros::NodeHandle * n);
//Process HGNSI Output Message based on 'messageID'
bool processMessage(uint8_t * buffer, uint32_t AddressID, uint32_t messageID);

//Control all generic publishers and scubscribers
void init_all_pubs(ros::NodeHandle * n); // if not used, pubs will be started automatically as needed
void init_all_subs(ros::NodeHandle * n);
void stop_all_pubs(void);
void stop_all_subs(void);

// Message 0x1001
#include <hg_nav_node/Msg_1001.h>
#define MSG_1001_PATH "HGuide/Input/0x1001_Enable_DisableOutputMessages"
void init_1001(ros::NodeHandle * n);
void stop_1001(void);
void convert(Msg_1001 messageIn, hg_nav_node::Msg_1001 * messageOut);
void convert(hg_nav_node::Msg_1001 messageIn, Msg_1001 * messageOut);
void Msg_1001_sub_callback(const hg_nav_node::Msg_1001::ConstPtr& Message);

// Message 0x1002
#include <hg_nav_node/Msg_1002.h>
#define MSG_1002_PATH "HGuide/Input/0x1002_CommandandControlMessage"
void init_1002(ros::NodeHandle * n);
void stop_1002(void);
void convert(Msg_1002 messageIn, hg_nav_node::Msg_1002 * messageOut);
void convert(hg_nav_node::Msg_1002 messageIn, Msg_1002 * messageOut);
void Msg_1002_sub_callback(const hg_nav_node::Msg_1002::ConstPtr& Message);

// Message 0x1003
#include <hg_nav_node/Msg_1003.h>
#define MSG_1003_PATH "HGuide/Input/0x1003_GPSInitializationandControl"
void init_1003(ros::NodeHandle * n);
void stop_1003(void);
void convert(Msg_1003 messageIn, hg_nav_node::Msg_1003 * messageOut);
void convert(hg_nav_node::Msg_1003 messageIn, Msg_1003 * messageOut);
void Msg_1003_sub_callback(const hg_nav_node::Msg_1003::ConstPtr& Message);

// Message 0x1004
#include <hg_nav_node/Msg_1004.h>
#define MSG_1004_PATH "HGuide/Input/0x1004_ConfigureInitializationInput"
void init_1004(ros::NodeHandle * n);
void stop_1004(void);
void convert(Msg_1004 messageIn, hg_nav_node::Msg_1004 * messageOut);
void convert(hg_nav_node::Msg_1004 messageIn, Msg_1004 * messageOut);
void Msg_1004_sub_callback(const hg_nav_node::Msg_1004::ConstPtr& Message);

// Message 0x1005
#include <hg_nav_node/Msg_1005.h>
#define MSG_1005_PATH "HGuide/Input/0x1005_PortConfigurationMessage"
void init_1005(ros::NodeHandle * n);
void stop_1005(void);
void convert(Msg_1005 messageIn, hg_nav_node::Msg_1005 * messageOut);
void convert(hg_nav_node::Msg_1005 messageIn, Msg_1005 * messageOut);
void Msg_1005_sub_callback(const hg_nav_node::Msg_1005::ConstPtr& Message);

// Message 0x1101
#include <hg_nav_node/Msg_1101.h>
#define MSG_1101_PATH "HGuide/Input/0x1101_InputMessageforBarometricAltitude"
void init_1101(ros::NodeHandle * n);
void stop_1101(void);
void convert(Msg_1101 messageIn, hg_nav_node::Msg_1101 * messageOut);
void convert(hg_nav_node::Msg_1101 messageIn, Msg_1101 * messageOut);
void Msg_1101_sub_callback(const hg_nav_node::Msg_1101::ConstPtr& Message);

// Message 0x1105
#include <hg_nav_node/Msg_1105.h>
#define MSG_1105_PATH "HGuide/Input/0x1105_InputMessageforMagneticHeading"
void init_1105(ros::NodeHandle * n);
void stop_1105(void);
void convert(Msg_1105 messageIn, hg_nav_node::Msg_1105 * messageOut);
void convert(hg_nav_node::Msg_1105 messageIn, Msg_1105 * messageOut);
void Msg_1105_sub_callback(const hg_nav_node::Msg_1105::ConstPtr& Message);

// Message 0x1108
#include <hg_nav_node/Msg_1108.h>
#define MSG_1108_PATH "HGuide/Input/0x1108_GPSPVTTMAidingInput"
void init_1108(ros::NodeHandle * n);
void stop_1108(void);
void convert(Msg_1108 messageIn, hg_nav_node::Msg_1108 * messageOut);
void convert(hg_nav_node::Msg_1108 messageIn, Msg_1108 * messageOut);
void Msg_1108_sub_callback(const hg_nav_node::Msg_1108::ConstPtr& Message);

// Message 0x1111
#include <hg_nav_node/Msg_1111.h>
#define MSG_1111_PATH "HGuide/Input/0x1111_ESpaceTrajectoryAiding"
void init_1111(ros::NodeHandle * n);
void stop_1111(void);
void convert(Msg_1111 messageIn, hg_nav_node::Msg_1111 * messageOut);
void convert(hg_nav_node::Msg_1111 messageIn, Msg_1111 * messageOut);
void Msg_1111_sub_callback(const hg_nav_node::Msg_1111::ConstPtr& Message);

// Message 0x1201
#include <hg_nav_node/Msg_1201.h>
#define MSG_1201_PATH "HGuide/Input/0x1201_TimeMarkPPSInput"
void init_1201(ros::NodeHandle * n);
void stop_1201(void);
void convert(Msg_1201 messageIn, hg_nav_node::Msg_1201 * messageOut);
void convert(hg_nav_node::Msg_1201 messageIn, Msg_1201 * messageOut);
void Msg_1201_sub_callback(const hg_nav_node::Msg_1201::ConstPtr& Message);

// Message 0x1401
#include <hg_nav_node/Msg_1401.h>
#define MSG_1401_PATH "HGuide/Input/0x1401_NavigationInputMessageforINS"
void init_1401(ros::NodeHandle * n);
void stop_1401(void);
void convert(Msg_1401 messageIn, hg_nav_node::Msg_1401 * messageOut);
void convert(hg_nav_node::Msg_1401 messageIn, Msg_1401 * messageOut);
void Msg_1401_sub_callback(const hg_nav_node::Msg_1401::ConstPtr& Message);

// Message 0x2001
#include <hg_nav_node/Msg_2001.h>
#define MSG_2001_PATH "HGuide/Output/0x2001_INSConfiguration"
void init_2001(ros::NodeHandle * n);
void stop_2001(void);
void convert(Msg_2001 messageIn, hg_nav_node::Msg_2001 * messageOut);
void convert(hg_nav_node::Msg_2001 messageIn, Msg_2001 * messageOut);
void Msg_2001_pub_callback(uint8_t *buffer);

// Message 0x2002
#include <hg_nav_node/Msg_2002.h>
#define MSG_2002_PATH "HGuide/Output/0x2002_GPSConfiguration"
void init_2002(ros::NodeHandle * n);
void stop_2002(void);
void convert(Msg_2002 messageIn, hg_nav_node::Msg_2002 * messageOut);
void convert(hg_nav_node::Msg_2002 messageIn, Msg_2002 * messageOut);
void Msg_2002_pub_callback(uint8_t *buffer);

// Message 0x2005
#include <hg_nav_node/Msg_2005.h>
#define MSG_2005_PATH "HGuide/Output/0x2005_PortConfigurationReplyMessage"
void init_2005(ros::NodeHandle * n);
void stop_2005(void);
void convert(Msg_2005 messageIn, hg_nav_node::Msg_2005 * messageOut);
void convert(hg_nav_node::Msg_2005 messageIn, Msg_2005 * messageOut);
void Msg_2005_pub_callback(uint8_t *buffer);

// Message 0x2011
#include <hg_nav_node/Msg_2011.h>
#define MSG_2011_PATH "HGuide/Output/0x2011_INSStatusInformation"
void init_2011(ros::NodeHandle * n);
void stop_2011(void);
void convert(Msg_2011 messageIn, hg_nav_node::Msg_2011 * messageOut);
void convert(hg_nav_node::Msg_2011 messageIn, Msg_2011 * messageOut);
void Msg_2011_pub_callback(uint8_t *buffer);

// Message 0x2021
#include <hg_nav_node/Msg_2021.h>
#define MSG_2021_PATH "HGuide/Output/0x2021_INSInitializationInfo"
void init_2021(ros::NodeHandle * n);
void stop_2021(void);
void convert(Msg_2021 messageIn, hg_nav_node::Msg_2021 * messageOut);
void convert(hg_nav_node::Msg_2021 messageIn, Msg_2021 * messageOut);
void Msg_2021_pub_callback(uint8_t *buffer);

// Message 0x20FF
#include <hg_nav_node/Msg_20FF.h>
#define MSG_20FF_PATH "HGuide/Output/0x20FF_ACK_NAKMessage"
void init_20FF(ros::NodeHandle * n);
void stop_20FF(void);
void convert(Msg_20FF messageIn, hg_nav_node::Msg_20FF * messageOut);
void convert(hg_nav_node::Msg_20FF messageIn, Msg_20FF * messageOut);
void Msg_20FF_pub_callback(uint8_t *buffer);

// Message 0x2201
#include <hg_nav_node/Msg_2201.h>
#define MSG_2201_PATH "HGuide/Output/0x2201_TimeMarkPPS"
void init_2201(ros::NodeHandle * n);
void stop_2201(void);
void convert(Msg_2201 messageIn, hg_nav_node::Msg_2201 * messageOut);
void convert(hg_nav_node::Msg_2201 messageIn, Msg_2201 * messageOut);
void Msg_2201_pub_callback(uint8_t *buffer);

// Message 0x2301
#include <hg_nav_node/Msg_2301.h>
#define MSG_2301_PATH "HGuide/Output/0x2301_AutopilotFlightControlInertialData"
void init_2301(ros::NodeHandle * n);
void stop_2301(void);
void convert(Msg_2301 messageIn, hg_nav_node::Msg_2301 * messageOut);
void convert(hg_nav_node::Msg_2301 messageIn, Msg_2301 * messageOut);
void Msg_2301_pub_callback(uint8_t *buffer);

// Message 0x2311
#include <hg_nav_node/Msg_2311.h>
#define MSG_2311_PATH "HGuide/Output/0x2311_UnfilteredInertialData"
void init_2311(ros::NodeHandle * n);
void stop_2311(void);
void convert(Msg_2311 messageIn, hg_nav_node::Msg_2311 * messageOut);
void convert(hg_nav_node::Msg_2311 messageIn, Msg_2311 * messageOut);
void Msg_2311_pub_callback(uint8_t *buffer);

// Message 0x2401
#include <hg_nav_node/Msg_2401.h>
#define MSG_2401_PATH "HGuide/Output/0x2401_Navigation"
void init_2401(ros::NodeHandle * n);
void stop_2401(void);
void convert(Msg_2401 messageIn, hg_nav_node::Msg_2401 * messageOut);
void convert(hg_nav_node::Msg_2401 messageIn, Msg_2401 * messageOut);
void Msg_2401_pub_callback(uint8_t *buffer);

// Message 0x2402
#include <hg_nav_node/Msg_2402.h>
#define MSG_2402_PATH "HGuide/Output/0x2402_SmoothedNavigation"
void init_2402(ros::NodeHandle * n);
void stop_2402(void);
void convert(Msg_2402 messageIn, hg_nav_node::Msg_2402 * messageOut);
void convert(hg_nav_node::Msg_2402 messageIn, Msg_2402 * messageOut);
void Msg_2402_pub_callback(uint8_t *buffer);

// Message 0x2411
#include <hg_nav_node/Msg_2411.h>
#define MSG_2411_PATH "HGuide/Output/0x2411_IMUErrorValues"
void init_2411(ros::NodeHandle * n);
void stop_2411(void);
void convert(Msg_2411 messageIn, hg_nav_node::Msg_2411 * messageOut);
void convert(hg_nav_node::Msg_2411 messageIn, Msg_2411 * messageOut);
void Msg_2411_pub_callback(uint8_t *buffer);

// Message 0x2421
#include <hg_nav_node/Msg_2421.h>
#define MSG_2421_PATH "HGuide/Output/0x2421_Kalmanfiltermeasurementstatus"
void init_2421(ros::NodeHandle * n);
void stop_2421(void);
void convert(Msg_2421 messageIn, hg_nav_node::Msg_2421 * messageOut);
void convert(hg_nav_node::Msg_2421 messageIn, Msg_2421 * messageOut);
void Msg_2421_pub_callback(uint8_t *buffer);

// Message 0x2422
#include <hg_nav_node/Msg_2422.h>
#define MSG_2422_PATH "HGuide/Output/0x2422_KFNavigationErrorValues"
void init_2422(ros::NodeHandle * n);
void stop_2422(void);
void convert(Msg_2422 messageIn, hg_nav_node::Msg_2422 * messageOut);
void convert(hg_nav_node::Msg_2422 messageIn, Msg_2422 * messageOut);
void Msg_2422_pub_callback(uint8_t *buffer);

// Message 0x2423
#include <hg_nav_node/Msg_2423.h>
#define MSG_2423_PATH "HGuide/Output/0x2423_IMUStandarddeviationValues"
void init_2423(ros::NodeHandle * n);
void stop_2423(void);
void convert(Msg_2423 messageIn, hg_nav_node::Msg_2423 * messageOut);
void convert(hg_nav_node::Msg_2423 messageIn, Msg_2423 * messageOut);
void Msg_2423_pub_callback(uint8_t *buffer);

// Message 0x2424
#include <hg_nav_node/Msg_2424.h>
#define MSG_2424_PATH "HGuide/Output/0x2424_GPSErrorEstimations"
void init_2424(ros::NodeHandle * n);
void stop_2424(void);
void convert(Msg_2424 messageIn, hg_nav_node::Msg_2424 * messageOut);
void convert(hg_nav_node::Msg_2424 messageIn, Msg_2424 * messageOut);
void Msg_2424_pub_callback(uint8_t *buffer);

// Message 0x2425
#include <hg_nav_node/Msg_2425.h>
#define MSG_2425_PATH "HGuide/Output/0x2425_TransferAlignMeasurementerrorsanduncertainties"
void init_2425(ros::NodeHandle * n);
void stop_2425(void);
void convert(Msg_2425 messageIn, hg_nav_node::Msg_2425 * messageOut);
void convert(hg_nav_node::Msg_2425 messageIn, Msg_2425 * messageOut);
void Msg_2425_pub_callback(uint8_t *buffer);

// Message 0x2426
#include <hg_nav_node/Msg_2426.h>
#define MSG_2426_PATH "HGuide/Output/0x2426_Barometererrorsanduncertainties"
void init_2426(ros::NodeHandle * n);
void stop_2426(void);
void convert(Msg_2426 messageIn, hg_nav_node::Msg_2426 * messageOut);
void convert(hg_nav_node::Msg_2426 messageIn, Msg_2426 * messageOut);
void Msg_2426_pub_callback(uint8_t *buffer);

// Message 0x2427
#include <hg_nav_node/Msg_2427.h>
#define MSG_2427_PATH "HGuide/Output/0x2427_Magnetometererrorsanduncertainties"
void init_2427(ros::NodeHandle * n);
void stop_2427(void);
void convert(Msg_2427 messageIn, hg_nav_node::Msg_2427 * messageOut);
void convert(hg_nav_node::Msg_2427 messageIn, Msg_2427 * messageOut);
void Msg_2427_pub_callback(uint8_t *buffer);

// Message 0x2428
#include <hg_nav_node/Msg_2428.h>
#define MSG_2428_PATH "HGuide/Output/0x2428_GNSS_PVT_PR_DR_MeasurementNormalizedResiduals"
void init_2428(ros::NodeHandle * n);
void stop_2428(void);
void convert(Msg_2428 messageIn, hg_nav_node::Msg_2428 * messageOut);
void convert(hg_nav_node::Msg_2428 messageIn, Msg_2428 * messageOut);
void Msg_2428_pub_callback(uint8_t *buffer);

// Message 0x2429
#include <hg_nav_node/Msg_2429.h>
#define MSG_2429_PATH "HGuide/Output/0x2429_TransferAlignmentNormalizedResiduals"
void init_2429(ros::NodeHandle * n);
void stop_2429(void);
void convert(Msg_2429 messageIn, hg_nav_node::Msg_2429 * messageOut);
void convert(hg_nav_node::Msg_2429 messageIn, Msg_2429 * messageOut);
void Msg_2429_pub_callback(uint8_t *buffer);

// Message 0x2501
#include <hg_nav_node/Msg_2501.h>
#define MSG_2501_PATH "HGuide/Output/0x2501_GPSChannelStatusOutput"
void init_2501(ros::NodeHandle * n);
void stop_2501(void);
void convert(Msg_2501 messageIn, hg_nav_node::Msg_2501 * messageOut);
void convert(hg_nav_node::Msg_2501 messageIn, Msg_2501 * messageOut);
void Msg_2501_pub_callback(uint8_t *buffer);

// Message 0x2611
#include <hg_nav_node/Msg_2611.h>
#define MSG_2611_PATH "HGuide/Output/0x2611_BuiltInTest_BIT_HistoryOutput"
void init_2611(ros::NodeHandle * n);
void stop_2611(void);
void convert(Msg_2611 messageIn, hg_nav_node::Msg_2611 * messageOut);
void convert(hg_nav_node::Msg_2611 messageIn, Msg_2611 * messageOut);
void Msg_2611_pub_callback(uint8_t *buffer);

// Message 0x4012
#include <hg_nav_node/Msg_4012.h>
#define MSG_4012_PATH "HGuide/Input/0x4012_CommandPowerCycleNavInitialization"
void init_4012(ros::NodeHandle * n);
void stop_4012(void);
void convert(Msg_4012 messageIn, hg_nav_node::Msg_4012 * messageOut);
void convert(hg_nav_node::Msg_4012 messageIn, Msg_4012 * messageOut);
void Msg_4012_sub_callback(const hg_nav_node::Msg_4012::ConstPtr& Message);

// Message 0x4109
#include <hg_nav_node/Msg_4109.h>
#define MSG_4109_PATH "HGuide/Input/0x4109_AttitudeInitializationMessage"
void init_4109(ros::NodeHandle * n);
void stop_4109(void);
void convert(Msg_4109 messageIn, hg_nav_node::Msg_4109 * messageOut);
void convert(hg_nav_node::Msg_4109 messageIn, Msg_4109 * messageOut);
void Msg_4109_sub_callback(const hg_nav_node::Msg_4109::ConstPtr& Message);

// Message 0x4110
#include <hg_nav_node/Msg_4110.h>
#define MSG_4110_PATH "HGuide/Input/0x4110_OdometerMeasurementInputMessage"
void init_4110(ros::NodeHandle * n);
void stop_4110(void);
void convert(Msg_4110 messageIn, hg_nav_node::Msg_4110 * messageOut);
void convert(hg_nav_node::Msg_4110 messageIn, Msg_4110 * messageOut);
void Msg_4110_sub_callback(const hg_nav_node::Msg_4110::ConstPtr& Message);

// Message 0x4201
#include <hg_nav_node/Msg_4201.h>
#define MSG_4201_PATH "HGuide/Input/0x4201_Event_InControlInputMessage"
void init_4201(ros::NodeHandle * n);
void stop_4201(void);
void convert(Msg_4201 messageIn, hg_nav_node::Msg_4201 * messageOut);
void convert(hg_nav_node::Msg_4201 messageIn, Msg_4201 * messageOut);
void Msg_4201_sub_callback(const hg_nav_node::Msg_4201::ConstPtr& Message);

// Message 0x4202
#include <hg_nav_node/Msg_4202.h>
#define MSG_4202_PATH "HGuide/Input/0x4202_Event_OutControlInputMessage"
void init_4202(ros::NodeHandle * n);
void stop_4202(void);
void convert(Msg_4202 messageIn, hg_nav_node::Msg_4202 * messageOut);
void convert(hg_nav_node::Msg_4202 messageIn, Msg_4202 * messageOut);
void Msg_4202_sub_callback(const hg_nav_node::Msg_4202::ConstPtr& Message);

// Message 0x4204
#include <hg_nav_node/Msg_4204.h>
#define MSG_4204_PATH "HGuide/Input/0x4204_Antennaleverarmssettings"
void init_4204(ros::NodeHandle * n);
void stop_4204(void);
void convert(Msg_4204 messageIn, hg_nav_node::Msg_4204 * messageOut);
void convert(hg_nav_node::Msg_4204 messageIn, Msg_4204 * messageOut);
void Msg_4204_sub_callback(const hg_nav_node::Msg_4204::ConstPtr& Message);

// Message 0x4401
#include <hg_nav_node/Msg_4401.h>
#define MSG_4401_PATH "HGuide/Input/0x4401_Position_Velocity_AttitudeInputMessage"
void init_4401(ros::NodeHandle * n);
void stop_4401(void);
void convert(Msg_4401 messageIn, hg_nav_node::Msg_4401 * messageOut);
void convert(hg_nav_node::Msg_4401 messageIn, Msg_4401 * messageOut);
void Msg_4401_sub_callback(const hg_nav_node::Msg_4401::ConstPtr& Message);

// Message 0x4404
#include <hg_nav_node/Msg_4404.h>
#define MSG_4404_PATH "HGuide/Input/0x4404_CasetoVehicleFrameSetUp"
void init_4404(ros::NodeHandle * n);
void stop_4404(void);
void convert(Msg_4404 messageIn, hg_nav_node::Msg_4404 * messageOut);
void convert(hg_nav_node::Msg_4404 messageIn, Msg_4404 * messageOut);
void Msg_4404_sub_callback(const hg_nav_node::Msg_4404::ConstPtr& Message);

// Message 0x4438
#include <hg_nav_node/Msg_4438.h>
#define MSG_4438_PATH "HGuide/Input/0x4438_DMIconfigurationmessage"
void init_4438(ros::NodeHandle * n);
void stop_4438(void);
void convert(Msg_4438 messageIn, hg_nav_node::Msg_4438 * messageOut);
void convert(hg_nav_node::Msg_4438 messageIn, Msg_4438 * messageOut);
void Msg_4438_sub_callback(const hg_nav_node::Msg_4438::ConstPtr& Message);

// Message 0x4651
#include <hg_nav_node/Msg_4651.h>
#define MSG_4651_PATH "HGuide/Input/0x4651_RealTimePostProcessingGoodToGoSetup"
void init_4651(ros::NodeHandle * n);
void stop_4651(void);
void convert(Msg_4651 messageIn, hg_nav_node::Msg_4651 * messageOut);
void convert(hg_nav_node::Msg_4651 messageIn, Msg_4651 * messageOut);
void Msg_4651_sub_callback(const hg_nav_node::Msg_4651::ConstPtr& Message);

// Message 0x4738
#include <hg_nav_node/Msg_4738.h>
#define MSG_4738_PATH "HGuide/Input/0x4738_DVLConfigurationInitialization"
void init_4738(ros::NodeHandle * n);
void stop_4738(void);
void convert(Msg_4738 messageIn, hg_nav_node::Msg_4738 * messageOut);
void convert(hg_nav_node::Msg_4738 messageIn, Msg_4738 * messageOut);
void Msg_4738_sub_callback(const hg_nav_node::Msg_4738::ConstPtr& Message);

// Message 0x5001
#include <hg_nav_node/Msg_5001.h>
#define MSG_5001_PATH "HGuide/Output/0x5001_GNSSReceiverVersionInformation"
void init_5001(ros::NodeHandle * n);
void stop_5001(void);
void convert(Msg_5001 messageIn, hg_nav_node::Msg_5001 * messageOut);
void convert(hg_nav_node::Msg_5001 messageIn, Msg_5001 * messageOut);
void Msg_5001_pub_callback(uint8_t *buffer);

// Message 0x5012
#include <hg_nav_node/Msg_5012.h>
#define MSG_5012_PATH "HGuide/Output/0x5012_GNSSReceiverHardwareStatusInformation"
void init_5012(ros::NodeHandle * n);
void stop_5012(void);
void convert(Msg_5012 messageIn, hg_nav_node::Msg_5012 * messageOut);
void convert(hg_nav_node::Msg_5012 messageIn, Msg_5012 * messageOut);
void Msg_5012_pub_callback(uint8_t *buffer);

// Message 0x5101
#include <hg_nav_node/Msg_5101.h>
#define MSG_5101_PATH "HGuide/Output/0x5101_GNSSRangeData"
void init_5101(ros::NodeHandle * n);
void stop_5101(void);
void convert(Msg_5101 messageIn, hg_nav_node::Msg_5101 * messageOut);
void convert(hg_nav_node::Msg_5101 messageIn, Msg_5101 * messageOut);
void Msg_5101_pub_callback(uint8_t *buffer);

// Message 0x5102
#include <hg_nav_node/Msg_5102.h>
#define MSG_5102_PATH "HGuide/Output/0x5102_GNSSSatellitePosition"
void init_5102(ros::NodeHandle * n);
void stop_5102(void);
void convert(Msg_5102 messageIn, hg_nav_node::Msg_5102 * messageOut);
void convert(hg_nav_node::Msg_5102 messageIn, Msg_5102 * messageOut);
void Msg_5102_pub_callback(uint8_t *buffer);

// Message 0x5103
#include <hg_nav_node/Msg_5103.h>
#define MSG_5103_PATH "HGuide/Output/0x5103_GPSEphemerisMessage"
void init_5103(ros::NodeHandle * n);
void stop_5103(void);
void convert(Msg_5103 messageIn, hg_nav_node::Msg_5103 * messageOut);
void convert(hg_nav_node::Msg_5103 messageIn, Msg_5103 * messageOut);
void Msg_5103_pub_callback(uint8_t *buffer);

// Message 0x5104
#include <hg_nav_node/Msg_5104.h>
#define MSG_5104_PATH "HGuide/Output/0x5104_GLONASSEphemerisMessage"
void init_5104(ros::NodeHandle * n);
void stop_5104(void);
void convert(Msg_5104 messageIn, hg_nav_node::Msg_5104 * messageOut);
void convert(hg_nav_node::Msg_5104 messageIn, Msg_5104 * messageOut);
void Msg_5104_pub_callback(uint8_t *buffer);

// Message 0x5105
#include <hg_nav_node/Msg_5105.h>
#define MSG_5105_PATH "HGuide/Output/0x5105_GALILEOEphemerisMessage"
void init_5105(ros::NodeHandle * n);
void stop_5105(void);
void convert(Msg_5105 messageIn, hg_nav_node::Msg_5105 * messageOut);
void convert(hg_nav_node::Msg_5105 messageIn, Msg_5105 * messageOut);
void Msg_5105_pub_callback(uint8_t *buffer);

// Message 0x5106
#include <hg_nav_node/Msg_5106.h>
#define MSG_5106_PATH "HGuide/Output/0x5106_BEIDOUEphemerisMessage"
void init_5106(ros::NodeHandle * n);
void stop_5106(void);
void convert(Msg_5106 messageIn, hg_nav_node::Msg_5106 * messageOut);
void convert(hg_nav_node::Msg_5106 messageIn, Msg_5106 * messageOut);
void Msg_5106_pub_callback(uint8_t *buffer);

// Message 0x5108
#include <hg_nav_node/Msg_5108.h>
#define MSG_5108_PATH "HGuide/Output/0x5108_GNSSReceiverPVTMeasurementOutput"
void init_5108(ros::NodeHandle * n);
void stop_5108(void);
void convert(Msg_5108 messageIn, hg_nav_node::Msg_5108 * messageOut);
void convert(hg_nav_node::Msg_5108 messageIn, Msg_5108 * messageOut);
void Msg_5108_pub_callback(uint8_t *buffer);

// Message 0x5109
#include <hg_nav_node/Msg_5109.h>
#define MSG_5109_PATH "HGuide/Output/0x5109_GNSSReceiverAttitude_HeadingMeasurementOutput"
void init_5109(ros::NodeHandle * n);
void stop_5109(void);
void convert(Msg_5109 messageIn, hg_nav_node::Msg_5109 * messageOut);
void convert(hg_nav_node::Msg_5109 messageIn, Msg_5109 * messageOut);
void Msg_5109_pub_callback(uint8_t *buffer);

// Message 0x5201
#include <hg_nav_node/Msg_5201.h>
#define MSG_5201_PATH "HGuide/Output/0x5201_PPSGNSSTime"
void init_5201(ros::NodeHandle * n);
void stop_5201(void);
void convert(Msg_5201 messageIn, hg_nav_node::Msg_5201 * messageOut);
void convert(hg_nav_node::Msg_5201 messageIn, Msg_5201 * messageOut);
void Msg_5201_pub_callback(uint8_t *buffer);

// Message 0x6001
#include <hg_nav_node/Msg_6001.h>
#define MSG_6001_PATH "HGuide/Output/0x6001_INSConfiguration"
void init_6001(ros::NodeHandle * n);
void stop_6001(void);
void convert(Msg_6001 messageIn, hg_nav_node::Msg_6001 * messageOut);
void convert(hg_nav_node::Msg_6001 messageIn, Msg_6001 * messageOut);
void Msg_6001_pub_callback(uint8_t *buffer);

// Message 0x6003
#include <hg_nav_node/Msg_6003.h>
#define MSG_6003_PATH "HGuide/Output/0x6003_HGuideSensorInstallationSettings"
void init_6003(ros::NodeHandle * n);
void stop_6003(void);
void convert(Msg_6003 messageIn, hg_nav_node::Msg_6003 * messageOut);
void convert(hg_nav_node::Msg_6003 messageIn, Msg_6003 * messageOut);
void Msg_6003_pub_callback(uint8_t *buffer);

// Message 0x6108
#include <hg_nav_node/Msg_6108.h>
#define MSG_6108_PATH "HGuide/Output/0x6108_GNSSReceiverPVTMeasurementOutput"
void init_6108(ros::NodeHandle * n);
void stop_6108(void);
void convert(Msg_6108 messageIn, hg_nav_node::Msg_6108 * messageOut);
void convert(hg_nav_node::Msg_6108 messageIn, Msg_6108 * messageOut);
void Msg_6108_pub_callback(uint8_t *buffer);

// Message 0x6109
#include <hg_nav_node/Msg_6109.h>
#define MSG_6109_PATH "HGuide/Output/0x6109_GNSSReceiverAttitude_HeadingMeasurementOutput"
void init_6109(ros::NodeHandle * n);
void stop_6109(void);
void convert(Msg_6109 messageIn, hg_nav_node::Msg_6109 * messageOut);
void convert(hg_nav_node::Msg_6109 messageIn, Msg_6109 * messageOut);
void Msg_6109_pub_callback(uint8_t *buffer);

// Message 0x6110
#include <hg_nav_node/Msg_6110.h>
#define MSG_6110_PATH "HGuide/Output/0x6110_OdometerMeasurementOutput"
void init_6110(ros::NodeHandle * n);
void stop_6110(void);
void convert(Msg_6110 messageIn, hg_nav_node::Msg_6110 * messageOut);
void convert(hg_nav_node::Msg_6110 messageIn, Msg_6110 * messageOut);
void Msg_6110_pub_callback(uint8_t *buffer);

// Message 0x6111
#include <hg_nav_node/Msg_6111.h>
#define MSG_6111_PATH "HGuide/Output/0x6111_ResultsofMotiondetectionalgorithminkalmanfilter"
void init_6111(ros::NodeHandle * n);
void stop_6111(void);
void convert(Msg_6111 messageIn, hg_nav_node::Msg_6111 * messageOut);
void convert(hg_nav_node::Msg_6111 messageIn, Msg_6111 * messageOut);
void Msg_6111_pub_callback(uint8_t *buffer);

// Message 0x6112
#include <hg_nav_node/Msg_6112.h>
#define MSG_6112_PATH "HGuide/Output/0x6112_VelocityHeadingOutput"
void init_6112(ros::NodeHandle * n);
void stop_6112(void);
void convert(Msg_6112 messageIn, hg_nav_node::Msg_6112 * messageOut);
void convert(hg_nav_node::Msg_6112 messageIn, Msg_6112 * messageOut);
void Msg_6112_pub_callback(uint8_t *buffer);

// Message 0x6201
#include <hg_nav_node/Msg_6201.h>
#define MSG_6201_PATH "HGuide/Output/0x6201_TimeMarkofEvent_In"
void init_6201(ros::NodeHandle * n);
void stop_6201(void);
void convert(Msg_6201 messageIn, hg_nav_node::Msg_6201 * messageOut);
void convert(hg_nav_node::Msg_6201 messageIn, Msg_6201 * messageOut);
void Msg_6201_pub_callback(uint8_t *buffer);

// Message 0x6202
#include <hg_nav_node/Msg_6202.h>
#define MSG_6202_PATH "HGuide/Output/0x6202_VehicleGeodeticPositionofEvent_In"
void init_6202(ros::NodeHandle * n);
void stop_6202(void);
void convert(Msg_6202 messageIn, hg_nav_node::Msg_6202 * messageOut);
void convert(hg_nav_node::Msg_6202 messageIn, Msg_6202 * messageOut);
void Msg_6202_pub_callback(uint8_t *buffer);

// Message 0x6203
#include <hg_nav_node/Msg_6203.h>
#define MSG_6203_PATH "HGuide/Output/0x6203_VehicleNEDVelocityofEvent_In"
void init_6203(ros::NodeHandle * n);
void stop_6203(void);
void convert(Msg_6203 messageIn, hg_nav_node::Msg_6203 * messageOut);
void convert(hg_nav_node::Msg_6203 messageIn, Msg_6203 * messageOut);
void Msg_6203_pub_callback(uint8_t *buffer);

// Message 0x6204
#include <hg_nav_node/Msg_6204.h>
#define MSG_6204_PATH "HGuide/Output/0x6204_VehicleEulerAttitudesofEvent_In"
void init_6204(ros::NodeHandle * n);
void stop_6204(void);
void convert(Msg_6204 messageIn, hg_nav_node::Msg_6204 * messageOut);
void convert(hg_nav_node::Msg_6204 messageIn, Msg_6204 * messageOut);
void Msg_6204_pub_callback(uint8_t *buffer);

// Message 0x6205
#include <hg_nav_node/Msg_6205.h>
#define MSG_6205_PATH "HGuide/Output/0x6205_VehicleAll_In_OnePos_Vel_AttofEvent_In"
void init_6205(ros::NodeHandle * n);
void stop_6205(void);
void convert(Msg_6205 messageIn, hg_nav_node::Msg_6205 * messageOut);
void convert(hg_nav_node::Msg_6205 messageIn, Msg_6205 * messageOut);
void Msg_6205_pub_callback(uint8_t *buffer);

// Message 0x6211
#include <hg_nav_node/Msg_6211.h>
#define MSG_6211_PATH "HGuide/Output/0x6211_TimeMarkofEvent_Out"
void init_6211(ros::NodeHandle * n);
void stop_6211(void);
void convert(Msg_6211 messageIn, hg_nav_node::Msg_6211 * messageOut);
void convert(hg_nav_node::Msg_6211 messageIn, Msg_6211 * messageOut);
void Msg_6211_pub_callback(uint8_t *buffer);

// Message 0x6311
#include <hg_nav_node/Msg_6311.h>
#define MSG_6311_PATH "HGuide/Output/0x6311_UnfilteredInertialDataUserSelectedReferenceFrame"
void init_6311(ros::NodeHandle * n);
void stop_6311(void);
void convert(Msg_6311 messageIn, hg_nav_node::Msg_6311 * messageOut);
void convert(hg_nav_node::Msg_6311 messageIn, Msg_6311 * messageOut);
void Msg_6311_pub_callback(uint8_t *buffer);

// Message 0x6403
#include <hg_nav_node/Msg_6403.h>
#define MSG_6403_PATH "HGuide/Output/0x6403_INSGeodeticPosition"
void init_6403(ros::NodeHandle * n);
void stop_6403(void);
void convert(Msg_6403 messageIn, hg_nav_node::Msg_6403 * messageOut);
void convert(hg_nav_node::Msg_6403 messageIn, Msg_6403 * messageOut);
void Msg_6403_pub_callback(uint8_t *buffer);

// Message 0x6405
#include <hg_nav_node/Msg_6405.h>
#define MSG_6405_PATH "HGuide/Output/0x6405_INSEulerAttitudes"
void init_6405(ros::NodeHandle * n);
void stop_6405(void);
void convert(Msg_6405 messageIn, hg_nav_node::Msg_6405 * messageOut);
void convert(hg_nav_node::Msg_6405 messageIn, Msg_6405 * messageOut);
void Msg_6405_pub_callback(uint8_t *buffer);

// Message 0x6406
#include <hg_nav_node/Msg_6406.h>
#define MSG_6406_PATH "HGuide/Output/0x6406_VehicleBodyRatesandLinearAccelerations"
void init_6406(ros::NodeHandle * n);
void stop_6406(void);
void convert(Msg_6406 messageIn, hg_nav_node::Msg_6406 * messageOut);
void convert(hg_nav_node::Msg_6406 messageIn, Msg_6406 * messageOut);
void Msg_6406_pub_callback(uint8_t *buffer);

// Message 0x6424
#include <hg_nav_node/Msg_6424.h>
#define MSG_6424_PATH "HGuide/Output/0x6424_GNSSMeasurementErrorEstimatesandUncertainty"
void init_6424(ros::NodeHandle * n);
void stop_6424(void);
void convert(Msg_6424 messageIn, hg_nav_node::Msg_6424 * messageOut);
void convert(hg_nav_node::Msg_6424 messageIn, Msg_6424 * messageOut);
void Msg_6424_pub_callback(uint8_t *buffer);

// Message 0x6428
#include <hg_nav_node/Msg_6428.h>
#define MSG_6428_PATH "HGuide/Output/0x6428_GNSSNormalizedMeasurementResiduals"
void init_6428(ros::NodeHandle * n);
void stop_6428(void);
void convert(Msg_6428 messageIn, hg_nav_node::Msg_6428 * messageOut);
void convert(hg_nav_node::Msg_6428 messageIn, Msg_6428 * messageOut);
void Msg_6428_pub_callback(uint8_t *buffer);

// Message 0x6438
#include <hg_nav_node/Msg_6438.h>
#define MSG_6438_PATH "HGuide/Output/0x6438_KalmanFilterCalibrationOfOdometerInputs"
void init_6438(ros::NodeHandle * n);
void stop_6438(void);
void convert(Msg_6438 messageIn, hg_nav_node::Msg_6438 * messageOut);
void convert(hg_nav_node::Msg_6438 messageIn, Msg_6438 * messageOut);
void Msg_6438_pub_callback(uint8_t *buffer);

// Message 0x6504
#include <hg_nav_node/Msg_6504.h>
#define MSG_6504_PATH "HGuide/Output/0x6504_INSNEDVelocity"
void init_6504(ros::NodeHandle * n);
void stop_6504(void);
void convert(Msg_6504 messageIn, hg_nav_node::Msg_6504 * messageOut);
void convert(hg_nav_node::Msg_6504 messageIn, Msg_6504 * messageOut);
void Msg_6504_pub_callback(uint8_t *buffer);

// Message 0x6505
#include <hg_nav_node/Msg_6505.h>
#define MSG_6505_PATH "HGuide/Output/0x6505_SkymapInformation"
void init_6505(ros::NodeHandle * n);
void stop_6505(void);
void convert(Msg_6505 messageIn, hg_nav_node::Msg_6505 * messageOut);
void convert(hg_nav_node::Msg_6505 messageIn, Msg_6505 * messageOut);
void Msg_6505_pub_callback(uint8_t *buffer);

// Message 0x6508
#include <hg_nav_node/Msg_6508.h>
#define MSG_6508_PATH "HGuide/Output/0x6508_AntennaConnectedInformation"
void init_6508(ros::NodeHandle * n);
void stop_6508(void);
void convert(Msg_6508 messageIn, hg_nav_node::Msg_6508 * messageOut);
void convert(hg_nav_node::Msg_6508 messageIn, Msg_6508 * messageOut);
void Msg_6508_pub_callback(uint8_t *buffer);

// Message 0x6601
#include <hg_nav_node/Msg_6601.h>
#define MSG_6601_PATH "HGuide/Output/0x6601_VirtualStatusLEDmessage"
void init_6601(ros::NodeHandle * n);
void stop_6601(void);
void convert(Msg_6601 messageIn, hg_nav_node::Msg_6601 * messageOut);
void convert(hg_nav_node::Msg_6601 messageIn, Msg_6601 * messageOut);
void Msg_6601_pub_callback(uint8_t *buffer);

// Message 0x6651
#include <hg_nav_node/Msg_6651.h>
#define MSG_6651_PATH "HGuide/Output/0x6651_RealTimePostProcessingGoodToGo"
void init_6651(ros::NodeHandle * n);
void stop_6651(void);
void convert(Msg_6651 messageIn, hg_nav_node::Msg_6651 * messageOut);
void convert(hg_nav_node::Msg_6651 messageIn, Msg_6651 * messageOut);
void Msg_6651_pub_callback(uint8_t *buffer);

// Message 0x6721
#include <hg_nav_node/Msg_6721.h>
#define MSG_6721_PATH "HGuide/Output/0x6721_NortekDVLBottomTrackData"
void init_6721(ros::NodeHandle * n);
void stop_6721(void);
void convert(Msg_6721 messageIn, hg_nav_node::Msg_6721 * messageOut);
void convert(hg_nav_node::Msg_6721 messageIn, Msg_6721 * messageOut);
void Msg_6721_pub_callback(uint8_t *buffer);

// Message 0x6722
#include <hg_nav_node/Msg_6722.h>
#define MSG_6722_PATH "HGuide/Output/0x6722_NortekDVLWaterTrackData"
void init_6722(ros::NodeHandle * n);
void stop_6722(void);
void convert(Msg_6722 messageIn, hg_nav_node::Msg_6722 * messageOut);
void convert(hg_nav_node::Msg_6722 messageIn, Msg_6722 * messageOut);
void Msg_6722_pub_callback(uint8_t *buffer);

// Message 0x6723
#include <hg_nav_node/Msg_6723.h>
#define MSG_6723_PATH "HGuide/Output/0x6723_NortekCurrentProfileData"
void init_6723(ros::NodeHandle * n);
void stop_6723(void);
void convert(Msg_6723 messageIn, hg_nav_node::Msg_6723 * messageOut);
void convert(hg_nav_node::Msg_6723 messageIn, Msg_6723 * messageOut);
void Msg_6723_pub_callback(uint8_t *buffer);

// Message 0x6724
#include <hg_nav_node/Msg_6724.h>
#define MSG_6724_PATH "HGuide/Output/0x6724_CurrentProfileVelocityData"
void init_6724(ros::NodeHandle * n);
void stop_6724(void);
void convert(Msg_6724 messageIn, hg_nav_node::Msg_6724 * messageOut);
void convert(hg_nav_node::Msg_6724 messageIn, Msg_6724 * messageOut);
void Msg_6724_pub_callback(uint8_t *buffer);

// Message 0x6725
#include <hg_nav_node/Msg_6725.h>
#define MSG_6725_PATH "HGuide/Output/0x6725_CurrentProfileAmplitudeData"
void init_6725(ros::NodeHandle * n);
void stop_6725(void);
void convert(Msg_6725 messageIn, hg_nav_node::Msg_6725 * messageOut);
void convert(hg_nav_node::Msg_6725 messageIn, Msg_6725 * messageOut);
void Msg_6725_pub_callback(uint8_t *buffer);

// Message 0x6726
#include <hg_nav_node/Msg_6726.h>
#define MSG_6726_PATH "HGuide/Output/0x6726_CurrentProfileCorrelationData"
void init_6726(ros::NodeHandle * n);
void stop_6726(void);
void convert(Msg_6726 messageIn, hg_nav_node::Msg_6726 * messageOut);
void convert(hg_nav_node::Msg_6726 messageIn, Msg_6726 * messageOut);
void Msg_6726_pub_callback(uint8_t *buffer);

// Message 0x6738
#include <hg_nav_node/Msg_6738.h>
#define MSG_6738_PATH "HGuide/Output/0x6738_DVLKalmanFilterData"
void init_6738(ros::NodeHandle * n);
void stop_6738(void);
void convert(Msg_6738 messageIn, hg_nav_node::Msg_6738 * messageOut);
void convert(hg_nav_node::Msg_6738 messageIn, Msg_6738 * messageOut);
void Msg_6738_pub_callback(uint8_t *buffer);

// Message 0x9900
#include <hg_nav_node/Msg_9900.h>
#define MSG_9900_PATH "HGuide/Output/0x9900_LogMessage"
void init_9900(ros::NodeHandle * n);
void stop_9900(void);
void convert(Msg_9900 messageIn, hg_nav_node::Msg_9900 * messageOut);
void convert(hg_nav_node::Msg_9900 messageIn, Msg_9900 * messageOut);
void Msg_9900_pub_callback(uint8_t *buffer);

// Message 0x9910
#include <hg_nav_node/Msg_9910.h>
#define MSG_9910_PATH "HGuide/Output/0x9910_ProcessorLoading"
void init_9910(ros::NodeHandle * n);
void stop_9910(void);
void convert(Msg_9910 messageIn, hg_nav_node::Msg_9910 * messageOut);
void convert(hg_nav_node::Msg_9910 messageIn, Msg_9910 * messageOut);
void Msg_9910_pub_callback(uint8_t *buffer);

// Message 0xCE01
#include <hg_nav_node/Msg_CE01.h>
#define MSG_CE01_PATH "HGuide/Output/0xCE01_HGuideConsoleCommand"
void init_CE01(ros::NodeHandle * n);
void stop_CE01(void);
void convert(Msg_CE01 messageIn, hg_nav_node::Msg_CE01 * messageOut);
void convert(hg_nav_node::Msg_CE01 messageIn, Msg_CE01 * messageOut);
void Msg_CE01_pub_callback(uint8_t *buffer);

// Message 0xFE01
#include <hg_nav_node/Msg_FE01.h>
#define MSG_FE01_PATH "HGuide/Output/0xFE01_ReadandWriteMemoryMessage"
void init_FE01(ros::NodeHandle * n);
void stop_FE01(void);
void convert(Msg_FE01 messageIn, hg_nav_node::Msg_FE01 * messageOut);
void convert(hg_nav_node::Msg_FE01 messageIn, Msg_FE01 * messageOut);
void Msg_FE01_pub_callback(uint8_t *buffer);

// Message 0xFE02
#include <hg_nav_node/Msg_FE02.h>
#define MSG_FE02_PATH "HGuide/Output/0xFE02_CustomerData_UniqueID"
void init_FE02(ros::NodeHandle * n);
void stop_FE02(void);
void convert(Msg_FE02 messageIn, hg_nav_node::Msg_FE02 * messageOut);
void convert(hg_nav_node::Msg_FE02 messageIn, Msg_FE02 * messageOut);
void Msg_FE02_pub_callback(uint8_t *buffer);

