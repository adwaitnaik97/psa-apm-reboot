#ifndef __HGuideAPI_Msg_1004_h__
#define __HGuideAPI_Msg_1004_h__
#pragma once

#include <cstdint>


// 0x1004 : Configure Initialization Input
//
// DO NOT USE. This message is deprecated, use 0x4404, 0x4204 messages instead
// 
class HGUIDE_DLL Msg_1004
{
public:
	Msg_1004();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 180;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1004; // Message ID
	static const uint32_t MessageLength = 45; // Message Length
	uint32_t Checksum; // Checksum
	bool ULV_GPSSigLatencyL1; // 0 = Use Default | 1 = Update
	bool ULV_GPSSigLatencyL2; // 0 = Use Default | 1 = Update
	
	// Configure Vehicle Frame Rotation
 	// 0 = No Change
 	// 1 = Given as DCM
 	// 2 = Given as Euler
	uint8_t ULV_VehFrametoCaseFrame;
	
	// Configure Vehicle Frame Lever Arms
	// 0 = No Change
	// 1 = Given as Vehicle to Case
	// 2 = Given as Case to Vehicle
	uint8_t ULV_Vehicle_Case_FrameLeverArm;
	
	// Configure Port4 to GPS Antenna Lever Arms
 	// 0 = No Change
 	// 1 = Given as Vehicle to Port4
 	// 2 = Given as Port4 to Vehicle
	uint8_t ULV_LeverArmToPort4GPSAnt;
	
	// Configure Measurement Frame Rotation
 	// 0 = No Change
 	// 1 = Given as DCM
 	// 2 = Given as Euler
	uint8_t ULV_MeasFrametoVehFrame;
	
	// Configure Measurement Frame Lever Arms
	// 0 = No Change
	// 1 = Given as Measurement to Nav Body in Vehicle Coordinates
	// 2 = Given as Measurement to Nav Body in Measurement Coordinates
	uint8_t ULV_LeverArmToICDNav;
	
	// Configure GPS Antenna Lever Arms
 	// 0 = No Change
 	// 1 = Given as Vehicle to ICD Body in Vehicle Coordinates
 	// 2 = Given as Vehicle to ICD Body in Measurement Coordinates
	uint8_t ULV_LeverArmToICDGPSAntenna;
	
	// Configure Location of Autopilot FC Data
 	// 0 = Use Default
 	// 1 = Use IMU Body Origin
 	// 2 = Use Case Frame Origin
 	// 3 = Use Vehicle Frame Origin
	uint8_t ULV_LocationOfAutopilotFCData;
	float L1SignalLatency;
	float L2SignalLatency;
	float Vehicle_to_Case_C11; // [C11] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C12; // [C12] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C13; // [C13] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C21; // [C21] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C22; // [C22] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C23; // [C23] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C31; // [C31] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C32; // [C32] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_C33; // [C33] Installation Attitude Vehicle to Case
	float Vehicle_to_Case_Roll; // [rad] Installation Roll Attitude Vehicle to Case
	float Vehicle_to_Case_Pitch; // [rad] Installation Pitch Attitude Vehicle to Case
	float Vehicle_to_Case_Heading; // [rad] Installation Heading Attitude Vehicle to Case
	float VehCaseLvrArm_x; // [m] Vehicle to Case Lever arm [x]
	float VehCaseLvrArm_y; // [m] Vehicle to Case Lever arm [y]
	float VehCaseLvrArm_z; // [m] Vehicle to Case Lever arm [z]
	float VehGPSPort4LvrArm_x;
	float VehGPSPort4LvrArm_y;
	float VehGPSPort4LvrArm_z;
	float Meas_to_Vehicle_C11; // [C11] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C12; // [C12] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C13; // [C13] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C21; // [C21] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C22; // [C22] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C23; // [C23] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C31; // [C31] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C32; // [C32] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_C33; // [C33] Installation Attitude Measurement to Vehicle as Direction Cosine Matrix
	float Meas_to_Vehicle_Roll; // [rad] Installation Roll Measurement to Vehicle as Euler Angles
	float Meas_to_Vehicle_Pitch; // [rad] Installation Pitch Measurement to Vehicle as Euler Angles
	float Meas_to_Vehicle_Heading; // [rad] Installation Heading Measurement to Vehicle as Euler Angles
	float VehICDNavLvrArm_x; // [m] HGNSI to Navigation Body Lever arm [x]
	float VehICDNavLvrArm_y; // [m] HGNSI to Navigation Body Lever arm [y]
	float VehICDNavLvrArm_z; // [m] HGNSI to Navigation Body Lever arm [z]
	float VehGPSPort4LvrArm_x_2; // [m] Internal GPS to Navigation Body Lever arm [x]
	float VehGPSPort4LvrArm_y_2; // [m] Internal GPS to Navigation Body Lever arm [y]
	float VehGPSPort4LvrArm_z_2; // [m] Internal GPS to Navigation Body Lever arm [z]
};

#endif // __HGuideAPI_Msg_1004_h__
