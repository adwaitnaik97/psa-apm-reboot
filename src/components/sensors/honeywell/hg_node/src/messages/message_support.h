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

// Message 0x01
#include <hg_node/Msg_01.h>
#define MSG_01_PATH "HGuide/Output/0x01_ControlMessage"
void init_01(ros::NodeHandle * n);
void stop_01(void);
void convert(Msg_01 messageIn, hg_node::Msg_01 * messageOut);
void convert(hg_node::Msg_01 messageIn, Msg_01 * messageOut);
void Msg_01_pub_callback(uint8_t *buffer);

// Message 0x02
#include <hg_node/Msg_02.h>
#define MSG_02_PATH "HGuide/Output/0x02_NavigationMessage"
void init_02(ros::NodeHandle * n);
void stop_02(void);
void convert(Msg_02 messageIn, hg_node::Msg_02 * messageOut);
void convert(hg_node::Msg_02 messageIn, Msg_02 * messageOut);
void Msg_02_pub_callback(uint8_t *buffer);

// Message 0x04
#include <hg_node/Msg_04.h>
#define MSG_04_PATH "HGuide/Output/0x04_ControlMessage"
void init_04(ros::NodeHandle * n);
void stop_04(void);
void convert(Msg_04 messageIn, hg_node::Msg_04 * messageOut);
void convert(hg_node::Msg_04 messageIn, Msg_04 * messageOut);
void Msg_04_pub_callback(uint8_t *buffer);

// Message 0x05
#include <hg_node/Msg_05.h>
#define MSG_05_PATH "HGuide/Output/0x05_NavigationMessage"
void init_05(ros::NodeHandle * n);
void stop_05(void);
void convert(Msg_05 messageIn, hg_node::Msg_05 * messageOut);
void convert(hg_node::Msg_05 messageIn, Msg_05 * messageOut);
void Msg_05_pub_callback(uint8_t *buffer);

// Message 0x0C
#include <hg_node/Msg_0C.h>
#define MSG_0C_PATH "HGuide/Output/0x0C_ControlMessage"
void init_0C(ros::NodeHandle * n);
void stop_0C(void);
void convert(Msg_0C messageIn, hg_node::Msg_0C * messageOut);
void convert(hg_node::Msg_0C messageIn, Msg_0C * messageOut);
void Msg_0C_pub_callback(uint8_t *buffer);

// Message 0x0D
#include <hg_node/Msg_0D.h>
#define MSG_0D_PATH "HGuide/Output/0x0D_NavigationMessage"
void init_0D(ros::NodeHandle * n);
void stop_0D(void);
void convert(Msg_0D messageIn, hg_node::Msg_0D * messageOut);
void convert(hg_node::Msg_0D messageIn, Msg_0D * messageOut);
void Msg_0D_pub_callback(uint8_t *buffer);

// Message 0xA1
#include <hg_node/Msg_A1.h>
#define MSG_A1_PATH "HGuide/Output/0xA1_ControlMessage"
void init_A1(ros::NodeHandle * n);
void stop_A1(void);
void convert(Msg_A1 messageIn, hg_node::Msg_A1 * messageOut);
void convert(hg_node::Msg_A1 messageIn, Msg_A1 * messageOut);
void Msg_A1_pub_callback(uint8_t *buffer);

// Message 0xA2
#include <hg_node/Msg_A2.h>
#define MSG_A2_PATH "HGuide/Output/0xA2_NavigationMessage"
void init_A2(ros::NodeHandle * n);
void stop_A2(void);
void convert(Msg_A2 messageIn, hg_node::Msg_A2 * messageOut);
void convert(hg_node::Msg_A2 messageIn, Msg_A2 * messageOut);
void Msg_A2_pub_callback(uint8_t *buffer);

// Message 0xA3
#include <hg_node/Msg_A3.h>
#define MSG_A3_PATH "HGuide/Output/0xA3_NavigationMessagewithoutControlData"
void init_A3(ros::NodeHandle * n);
void stop_A3(void);
void convert(Msg_A3 messageIn, hg_node::Msg_A3 * messageOut);
void convert(hg_node::Msg_A3 messageIn, Msg_A3 * messageOut);
void Msg_A3_pub_callback(uint8_t *buffer);

// Message 0xA9
#include <hg_node/Msg_A9.h>
#define MSG_A9_PATH "HGuide/Output/0xA9_NavigationMessagewithmagnetometerflux"
void init_A9(ros::NodeHandle * n);
void stop_A9(void);
void convert(Msg_A9 messageIn, hg_node::Msg_A9 * messageOut);
void convert(hg_node::Msg_A9 messageIn, Msg_A9 * messageOut);
void Msg_A9_pub_callback(uint8_t *buffer);

// Message 0xAC
#include <hg_node/Msg_AC.h>
#define MSG_AC_PATH "HGuide/Output/0xAC_ControlMessagewithMagnetometerdata"
void init_AC(ros::NodeHandle * n);
void stop_AC(void);
void convert(Msg_AC messageIn, hg_node::Msg_AC * messageOut);
void convert(hg_node::Msg_AC messageIn, Msg_AC * messageOut);
void Msg_AC_pub_callback(uint8_t *buffer);

// Message 0xAD
#include <hg_node/Msg_AD.h>
#define MSG_AD_PATH "HGuide/Output/0xAD_NavigationMessagewithMagnetometerdata"
void init_AD(ros::NodeHandle * n);
void stop_AD(void);
void convert(Msg_AD messageIn, hg_node::Msg_AD * messageOut);
void convert(hg_node::Msg_AD messageIn, Msg_AD * messageOut);
void Msg_AD_pub_callback(uint8_t *buffer);

// Message 0xAE
#include <hg_node/Msg_AE.h>
#define MSG_AE_PATH "HGuide/Output/0xAE_NavigationMessagewithMagnetometerdataandwithoutControlData"
void init_AE(ros::NodeHandle * n);
void stop_AE(void);
void convert(Msg_AE messageIn, hg_node::Msg_AE * messageOut);
void convert(hg_node::Msg_AE messageIn, Msg_AE * messageOut);
void Msg_AE_pub_callback(uint8_t *buffer);

// Message 0xB0
#include <hg_node/Msg_B0.h>
#define MSG_B0_PATH "HGuide/Output/0xB0_1HzStatusMessage"
void init_B0(ros::NodeHandle * n);
void stop_B0(void);
void convert(Msg_B0 messageIn, hg_node::Msg_B0 * messageOut);
void convert(hg_node::Msg_B0 messageIn, Msg_B0 * messageOut);
void Msg_B0_pub_callback(uint8_t *buffer);

// Message 0xB1
#include <hg_node/Msg_B1.h>
#define MSG_B1_PATH "HGuide/Output/0xB1_ControlMessage"
void init_B1(ros::NodeHandle * n);
void stop_B1(void);
void convert(Msg_B1 messageIn, hg_node::Msg_B1 * messageOut);
void convert(hg_node::Msg_B1 messageIn, Msg_B1 * messageOut);
void Msg_B1_pub_callback(uint8_t *buffer);

// Message 0xB2
#include <hg_node/Msg_B2.h>
#define MSG_B2_PATH "HGuide/Output/0xB2_GuidanceMessage"
void init_B2(ros::NodeHandle * n);
void stop_B2(void);
void convert(Msg_B2 messageIn, hg_node::Msg_B2 * messageOut);
void convert(hg_node::Msg_B2 messageIn, Msg_B2 * messageOut);
void Msg_B2_pub_callback(uint8_t *buffer);

// Message 0xB3
#include <hg_node/Msg_B3.h>
#define MSG_B3_PATH "HGuide/Output/0xB3_MagnetometerMessage"
void init_B3(ros::NodeHandle * n);
void stop_B3(void);
void convert(Msg_B3 messageIn, hg_node::Msg_B3 * messageOut);
void convert(hg_node::Msg_B3 messageIn, Msg_B3 * messageOut);
void Msg_B3_pub_callback(uint8_t *buffer);

// Message 0xB4
#include <hg_node/Msg_B4.h>
#define MSG_B4_PATH "HGuide/Output/0xB4_AttitudeMessage"
void init_B4(ros::NodeHandle * n);
void stop_B4(void);
void convert(Msg_B4 messageIn, hg_node::Msg_B4 * messageOut);
void convert(hg_node::Msg_B4 messageIn, Msg_B4 * messageOut);
void Msg_B4_pub_callback(uint8_t *buffer);

// Message 0xCA
#include <hg_node/Msg_CA.h>
#define MSG_CA_PATH "HGuide/Output/0xCA_ControlMessage"
void init_CA(ros::NodeHandle * n);
void stop_CA(void);
void convert(Msg_CA messageIn, hg_node::Msg_CA * messageOut);
void convert(hg_node::Msg_CA messageIn, Msg_CA * messageOut);
void Msg_CA_pub_callback(uint8_t *buffer);

// Message 0xF0
#include <hg_node/Msg_F0.h>
#define MSG_F0_PATH "HGuide/Output/0xF0_StatusMessage_DeviceType"
void init_F0(ros::NodeHandle * n);
void stop_F0(void);
void convert(Msg_F0 messageIn, hg_node::Msg_F0 * messageOut);
void convert(hg_node::Msg_F0 messageIn, Msg_F0 * messageOut);
void Msg_F0_pub_callback(uint8_t *buffer);

// Message 0xF1
#include <hg_node/Msg_F1.h>
#define MSG_F1_PATH "HGuide/Output/0xF1_StatusMessage_SerialNumber"
void init_F1(ros::NodeHandle * n);
void stop_F1(void);
void convert(Msg_F1 messageIn, hg_node::Msg_F1 * messageOut);
void convert(hg_node::Msg_F1 messageIn, Msg_F1 * messageOut);
void Msg_F1_pub_callback(uint8_t *buffer);

// Message 0xF2
#include <hg_node/Msg_F2.h>
#define MSG_F2_PATH "HGuide/Output/0xF2_StatusMessage_HardwareInformation"
void init_F2(ros::NodeHandle * n);
void stop_F2(void);
void convert(Msg_F2 messageIn, hg_node::Msg_F2 * messageOut);
void convert(hg_node::Msg_F2 messageIn, Msg_F2 * messageOut);
void Msg_F2_pub_callback(uint8_t *buffer);

// Message 0xF3
#include <hg_node/Msg_F3.h>
#define MSG_F3_PATH "HGuide/Output/0xF3_StatusMessage_SoftwareInformation"
void init_F3(ros::NodeHandle * n);
void stop_F3(void);
void convert(Msg_F3 messageIn, hg_node::Msg_F3 * messageOut);
void convert(hg_node::Msg_F3 messageIn, Msg_F3 * messageOut);
void Msg_F3_pub_callback(uint8_t *buffer);

// Message 0xF4
#include <hg_node/Msg_F4.h>
#define MSG_F4_PATH "HGuide/Output/0xF4_StatusMessage_PortInformation"
void init_F4(ros::NodeHandle * n);
void stop_F4(void);
void convert(Msg_F4 messageIn, hg_node::Msg_F4 * messageOut);
void convert(hg_node::Msg_F4 messageIn, Msg_F4 * messageOut);
void Msg_F4_pub_callback(uint8_t *buffer);

// Message 0xF5
#include <hg_node/Msg_F5.h>
#define MSG_F5_PATH "HGuide/Output/0xF5_StatusMessage_GyroConfiguration"
void init_F5(ros::NodeHandle * n);
void stop_F5(void);
void convert(Msg_F5 messageIn, hg_node::Msg_F5 * messageOut);
void convert(hg_node::Msg_F5 messageIn, Msg_F5 * messageOut);
void Msg_F5_pub_callback(uint8_t *buffer);

// Message 0xF6
#include <hg_node/Msg_F6.h>
#define MSG_F6_PATH "HGuide/Output/0xF6_StatusMessage_AccelConfiguration"
void init_F6(ros::NodeHandle * n);
void stop_F6(void);
void convert(Msg_F6 messageIn, hg_node::Msg_F6 * messageOut);
void convert(hg_node::Msg_F6 messageIn, Msg_F6 * messageOut);
void Msg_F6_pub_callback(uint8_t *buffer);

// Message 0xF7
#include <hg_node/Msg_F7.h>
#define MSG_F7_PATH "HGuide/Output/0xF7_StatusMessage_OperationInformation"
void init_F7(ros::NodeHandle * n);
void stop_F7(void);
void convert(Msg_F7 messageIn, hg_node::Msg_F7 * messageOut);
void convert(hg_node::Msg_F7 messageIn, Msg_F7 * messageOut);
void Msg_F7_pub_callback(uint8_t *buffer);

// Message 0xF8
#include <hg_node/Msg_F8.h>
#define MSG_F8_PATH "HGuide/Output/0xF8_StatusMessage_PowerInformation"
void init_F8(ros::NodeHandle * n);
void stop_F8(void);
void convert(Msg_F8 messageIn, hg_node::Msg_F8 * messageOut);
void convert(hg_node::Msg_F8 messageIn, Msg_F8 * messageOut);
void Msg_F8_pub_callback(uint8_t *buffer);

// Message 0xF9
#include <hg_node/Msg_F9.h>
#define MSG_F9_PATH "HGuide/Output/0xF9_StatusMessage_CPUUtilization"
void init_F9(ros::NodeHandle * n);
void stop_F9(void);
void convert(Msg_F9 messageIn, hg_node::Msg_F9 * messageOut);
void convert(hg_node::Msg_F9 messageIn, Msg_F9 * messageOut);
void Msg_F9_pub_callback(uint8_t *buffer);

// Message 0xFA
#include <hg_node/Msg_FA.h>
#define MSG_FA_PATH "HGuide/Output/0xFA_StatusMessage_SensorHealth"
void init_FA(ros::NodeHandle * n);
void stop_FA(void);
void convert(Msg_FA messageIn, hg_node::Msg_FA * messageOut);
void convert(hg_node::Msg_FA messageIn, Msg_FA * messageOut);
void Msg_FA_pub_callback(uint8_t *buffer);

// Message 0xFB
#include <hg_node/Msg_FB.h>
#define MSG_FB_PATH "HGuide/Output/0xFB_StatusMessage_BITHealth"
void init_FB(ros::NodeHandle * n);
void stop_FB(void);
void convert(Msg_FB messageIn, hg_node::Msg_FB * messageOut);
void convert(hg_node::Msg_FB messageIn, Msg_FB * messageOut);
void Msg_FB_pub_callback(uint8_t *buffer);

