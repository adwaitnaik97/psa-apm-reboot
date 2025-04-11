#ifndef __HGuideAPI_Msg_2001_h__
#define __HGuideAPI_Msg_2001_h__
#pragma once

#include <cstdint>


// 0x2001 : INS Configuration
//
// Message contains a complete configuration of the device.
// Use message 0x6001 for information on SN and SW versions
// Use the 0x1001 message to leave this on at 0.1 Hz (enabled; default), or to have this only produced and sent once (disabled).
// 
class HGUIDE_DLL Msg_2001
{
public:
	Msg_2001();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 320;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2001; // Message ID
	static const uint32_t MessageLength = 80; // Message Length
	uint32_t Checksum; // Checksum
	uint8_t IMUSerialNumber[8]; // INS Device Serial Number
	uint8_t ISA_Performance_Grade[8]; // INS Performance Grade
	uint8_t IMUSoftwareVersion[16]; // IMU Software Version
	uint8_t FPGA_VERSION[16]; // FPGA
	uint8_t FSBL_VERSION[16]; // First Stage Bootloader
	uint8_t SSBL_VERSION[16]; // Second Stage Bootloader
	uint8_t HGuideSoftwareVersion[16]; // Application Software Version
	uint8_t HGuideSoftwareVersionBuildDate[16]; // Application Software Build Date
	uint8_t HGuideSoftwareVersionBuildTime[16]; // Application Software Build Time
	uint8_t INS_CONFIG_FILEID[4]; // INC Configuration File ID
	uint8_t SSBL_REGINIT_FILEID[4]; // Reginit File ID
	uint8_t SSBL_FLASHLDR_FILEID[4]; // Flash loader File ID
	uint8_t HGuideSerialNumber[10]; // RESERVED : Not populated
	uint8_t HGuidePartNumber[10]; // RESERVED : Not populated
	uint32_t device_configuration;
	float CaseToNavLeverArmX; // [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [x]
	float CaseToNavLeverArmY; // [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [y]
	float CaseToNavLeverArmZ; // [m] Case Origin to Nav Body Origin Lever Arm in Case Frame [z]
	float CaseToNavQuaternionS; // Case Frame to Nav Body Quaternion [s]
	float CaseToNavQuaternionI; // Case Frame to Nav Body Quaternion [i]
	float CaseToNavQuaternionJ; // Case Frame to Nav Body Quaternion [j]
	float CaseToNavQuaternionK; // Case Frame to Nav Body Quaternion [k]
	float VehicleLeverArmX; // [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [x]
	float VehicleLeverArmY; // [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [y]
	float VehicleLeverArmZ; // [m] Vehicle Origin to Case Origin Lever Arm in Vehicle Frame [z]
	float VehicleQuaterionS; // Case Frame to Vehicle Frame Quaternion [s]
	float VehicleQuaterionI; // Case Frame to Vehicle Frame Quaternion [i]
	float VehicleQuaterionJ; // Case Frame to Vehicle Frame Quaternion [j]
	float VehicleQuaterionK; // Case Frame to Vehicle Frame Quaternion [k]
	float RF1AntennaLeverArmRSS; // Root Sum Squared of the Lever-arm estimated uncertainty. This will be used to determine if the lever-arm estimation function is on
	float RF1AntennaLeverArmX; // [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [x]
	float RF1AntennaLeverArmY; // [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [y]
	float RF1AntennaLeverArmZ; // [m] Vehicle Origin to Integrated GPSR Main Antenna Origin Lever Arm in Vehicle Frame [z]
	float HGNSI_AntennaLeverArmX; // RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [x]
	float HGNSI_AntennaLeverArmY; // RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [y]
	float HGNSI_AntennaLeverArmZ; // RESERVED [m] Vehicle Origin to HGNSI GPSR Antenna Origin Lever Arm in HGNSI frame [z]
	float RF2AntennaLeverArmX; // [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [x]
	float RF2AntennaLeverArmY; // [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [y]
	float RF2AntennaLeverArmZ; // [m] Vehicle Origin to Integrated GPSR Auxiliary Antenna Origin Lever Arm in Case Frame [z]
	float GPSantToCaseQuaternionS; // GPS Antenna to Integrated Case Frame Quaternion [s]
	float GPSantToCaseQuaternionI; // GPS Antenna to Integrated Case Frame Quaternion [i]
	float GPSantToCaseQuaternionJ; // GPS Antenna to Integrated Case Frame Quaternion [j]
	float GPSantToCaseQuaternionK; // GPS Antenna to Integrated Case Frame Quaternion [k]
};

#endif // __HGuideAPI_Msg_2001_h__
