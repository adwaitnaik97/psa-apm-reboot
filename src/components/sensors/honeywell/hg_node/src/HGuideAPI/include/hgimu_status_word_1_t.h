#ifndef __HGuideAPI_hgimu_status_word_1_t_h__
#define __HGuideAPI_hgimu_status_word_1_t_h__
#pragma once

#include <include/control_frequency_t.h>
#include <include/guidance_frequency_t.h>

//
// Unit status and output data rates are specified in
// the HGuide IMU Status Word #1 
//
struct hgimu_status_word_1_t
{
	
	// Bits 0-3 : 4-bit Counter / Status Word 2 ID
 	// Where:
 	// xx00=2A Word Transmitted this Message
 	// xx01=2B Word Transmitted this Message
 	// xx10=2C Word Transmitted this Message
 	// xx11=2D Word Transmitted this Message
	uint8_t StatusWord2ID;
	control_frequency_t Control_Frequency; // Output frequency of Control message
	guidance_frequency_t Guidance_Frequency; // Output frequency of Guidance message
	bool Gyro_BIT_Summary; // Gyro BIT Summary 0=OK, 1=Fail
	bool Accelerometer_BIT_Summary; // Accelerometer BIT Summary 0=OK, 1=Fail
	bool Magnetometer_BIT_Summary; // Magnetometer BIT Summary 0=OK, 1=Fail
	bool CBIT_Status; // CBIT Status 0=OK, 1=Fail

	void Default()
	{
		StatusWord2ID = 0;
		Control_Frequency = MSG_CTRL_0_RATE;
		Guidance_Frequency = MSG_GUID_0_RATE;
		Gyro_BIT_Summary = 0;
		Accelerometer_BIT_Summary = 0;
		Magnetometer_BIT_Summary = 0;
		CBIT_Status = 0;
	}
};

#endif // __HGuideAPI_hgimu_status_word_1_t_h__
