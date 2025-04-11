#ifndef __HGuideAPI_master_h__
#define __HGuideAPI_master_h__
#pragma once

#ifdef _WIN32
	#ifdef COMPILE_DLL
		#define HGUIDE_DLL __declspec(dllexport)
	#else
		#define HGUIDE_DLL __declspec(dllimport)
	#endif
#else // UNIX
	#define HGUIDE_DLL
#endif // _WIN32

// MACRO Functions to check the range of input types for overflow and a special case of NaN
#define CHECK_MAX_INT8(x)   ((x == 0x7fffffff) ? 0 : ((x > INT8_MAX) ? INT8_MAX : ((x < INT8_MIN) ? INT8_MIN : x)))
#define CHECK_MAX_UINT8(x)  ((x == 0x7fffffff) ? 0 : ((x > UINT8_MAX) ? UINT8_MAX : ((x < 0) ? 0 : x)))
#define CHECK_MAX_INT16(x)  ((x == 0x7fffffff) ? 0 : ((x > INT16_MAX) ? INT16_MAX : ((x < INT32_MIN) ? INT32_MIN : x)))
#define CHECK_MAX_UINT16(x) ((x == 0x7fffffff) ? 0 : ((x > UINT16_MAX) ? UINT16_MAX : ((x < 0) ? 0 : x)))
#define CHECK_MAX_INT32(x)  ((x == 0x7fffffff) ? 0 : ((x > INT32_MAX) ? INT32_MAX : ((x < INT32_MIN) ? INT32_MIN : x)))
#define CHECK_MAX_UINT32(x) ((x == 0x7fffffff) ? 0 : ((x > UINT32_MAX) ? UINT32_MAX : ((x < 0) ? 0 : x)))
#define CHECK_MAX_INT64(x)  ((x == 0x7fffffff) ? 0 : ((x > INT64_MAX) ? INT64_MAX : ((x < INT64_MIN) ? INT64_MIN : x)))
#define CHECK_MAX_UINT64(x) ((x == 0x7fffffff) ? 0 : ((x > UINT64_MAX) ? UINT64_MAX : ((x < 0) ? 0 : x)))

// Select the endianness of serialized messages
// Options:
//    littleEndian
//    bigEndian
#define HGuideAPI_endianness littleEndian

#include <include/utils/Checksum.h>


#include <include/Msg_1001.h> // 0x1001 : Enable/Disable Output Messages
#include <include/Msg_1002.h> // 0x1002 : Command and Control Message
#include <include/Msg_1003.h> // 0x1003 : GPS Initialization and Control
#include <include/Msg_1004.h> // 0x1004 : Configure Initialization Input
#include <include/Msg_1005.h> // 0x1005 : Port Configuration Message
#include <include/Msg_1101.h> // 0x1101 : Input Message for Barometric Altitude
#include <include/Msg_1105.h> // 0x1105 : Input Message for Magnetic Heading
#include <include/Msg_1108.h> // 0x1108 : GPS PVT TM Aiding Input
#include <include/Msg_1111.h> // 0x1111 : ESpace Trajectory Aiding
#include <include/Msg_1201.h> // 0x1201 : Time Mark PPS Input
#include <include/Msg_1401.h> // 0x1401 : Navigation Input Message for INS
#include <include/Msg_2001.h> // 0x2001 : INS Configuration
#include <include/Msg_2002.h> // 0x2002 : GPS Configuration
#include <include/Msg_2005.h> // 0x2005 : Port Configuration Reply Message
#include <include/Msg_2011.h> // 0x2011 : INS Status Information
#include <include/Msg_2021.h> // 0x2021 : INS Initialization Info
#include <include/Msg_20FF.h> // 0x20FF : ACK/NAK Message
#include <include/Msg_2201.h> // 0x2201 : Time Mark PPS
#include <include/Msg_2301.h> // 0x2301 : Autopilot Flight Control Inertial Data
#include <include/Msg_2311.h> // 0x2311 : Unfiltered Inertial Data
#include <include/Msg_2401.h> // 0x2401 : Navigation
#include <include/Msg_2402.h> // 0x2402 : Smoothed Navigation
#include <include/Msg_2411.h> // 0x2411 : IMU Error Values
#include <include/Msg_2421.h> // 0x2421 : Kalman filter measurement status
#include <include/Msg_2422.h> // 0x2422 : KF Navigation Error Values
#include <include/Msg_2423.h> // 0x2423 : IMU Standard deviation Values
#include <include/Msg_2424.h> // 0x2424 : GPS Error Estimations
#include <include/Msg_2425.h> // 0x2425 : Transfer Align Measurement errors and uncertainties
#include <include/Msg_2426.h> // 0x2426 : Barometer errors and uncertainties
#include <include/Msg_2427.h> // 0x2427 : Magnetometer errors and uncertainties
#include <include/Msg_2428.h> // 0x2428 : GNSS (PVT, PR/DR) Measurement Normalized Residuals
#include <include/Msg_2429.h> // 0x2429 : Transfer Alignment Normalized Residuals
#include <include/Msg_2501.h> // 0x2501 : GPS Channel Status Output
#include <include/Msg_2611.h> // 0x2611 : Built In Test (BIT) History Output
#include <include/Msg_4012.h> // 0x4012 : Command Power Cycle Nav Initialization
#include <include/Msg_4109.h> // 0x4109 : Attitude Initialization Message
#include <include/Msg_4110.h> // 0x4110 : Odometer Measurement Input Message
#include <include/Msg_4201.h> // 0x4201 : Event-In Control Input Message
#include <include/Msg_4202.h> // 0x4202 : Event-Out Control Input Message
#include <include/Msg_4204.h> // 0x4204 : Antenna lever arms settings
#include <include/Msg_4401.h> // 0x4401 : Position, Velocity, Attitude Input Message
#include <include/Msg_4404.h> // 0x4404 : Case to Vehicle Frame Set Up
#include <include/Msg_4438.h> // 0x4438 : DMI configuration message
#include <include/Msg_4651.h> // 0x4651 : Real Time Post Processing Good To Go Setup
#include <include/Msg_4738.h> // 0x4738 : DVL Configuration Initialization
#include <include/Msg_5001.h> // 0x5001 : GNSS Receiver Version Information
#include <include/Msg_5012.h> // 0x5012 : GNSS Receiver Hardware Status Information
#include <include/Msg_5101.h> // 0x5101 : GNSS Range Data
#include <include/Msg_5102.h> // 0x5102 : GNSS Satellite Position
#include <include/Msg_5103.h> // 0x5103 : GPS Ephemeris Message
#include <include/Msg_5104.h> // 0x5104 : GLONASS Ephemeris Message
#include <include/Msg_5105.h> // 0x5105 : GALILEO Ephemeris Message
#include <include/Msg_5106.h> // 0x5106 : BEIDOU Ephemeris Message
#include <include/Msg_5108.h> // 0x5108 : GNSS Receiver PVT Measurement Output
#include <include/Msg_5109.h> // 0x5109 : GNSS Receiver Attitude/Heading Measurement Output
#include <include/Msg_5201.h> // 0x5201 : PPS GNSS Time
#include <include/Msg_6001.h> // 0x6001 : INS Configuration
#include <include/Msg_6003.h> // 0x6003 : HGuide Sensor Installation Settings
#include <include/Msg_6108.h> // 0x6108 : GNSS Receiver PVT Measurement Output
#include <include/Msg_6109.h> // 0x6109 : GNSS Receiver Attitude/Heading Measurement Output
#include <include/Msg_6110.h> // 0x6110 : Odometer Measurement Output
#include <include/Msg_6111.h> // 0x6111 : Results of Motion detection algorithm in kalman filter
#include <include/Msg_6112.h> // 0x6112 : Velocity Heading Output
#include <include/Msg_6201.h> // 0x6201 : Time Mark of Event-In
#include <include/Msg_6202.h> // 0x6202 : Vehicle Geodetic Position of Event-In
#include <include/Msg_6203.h> // 0x6203 : Vehicle NED Velocity of Event-In
#include <include/Msg_6204.h> // 0x6204 : Vehicle Euler Attitudes of Event-In
#include <include/Msg_6205.h> // 0x6205 : Vehicle All-In-One Pos, Vel, Att of Event-In
#include <include/Msg_6211.h> // 0x6211 : Time Mark of Event-Out
#include <include/Msg_6311.h> // 0x6311 : Unfiltered Inertial Data User Selected Reference Frame
#include <include/Msg_6403.h> // 0x6403 : INS Geodetic Position
#include <include/Msg_6405.h> // 0x6405 : INS Euler Attitudes
#include <include/Msg_6406.h> // 0x6406 : Vehicle Body Rates and Linear Accelerations
#include <include/Msg_6424.h> // 0x6424 : GNSS Measurement Error Estimates and Uncertainty
#include <include/Msg_6428.h> // 0x6428 : GNSS Normalized Measurement Residuals
#include <include/Msg_6438.h> // 0x6438 : Kalman Filter Calibration Of Odometer Inputs
#include <include/Msg_6504.h> // 0x6504 : INS NED Velocity
#include <include/Msg_6505.h> // 0x6505 : Skymap Information
#include <include/Msg_6508.h> // 0x6508 : Antenna Connected Information
#include <include/Msg_6601.h> // 0x6601 : Virtual Status LED message
#include <include/Msg_6651.h> // 0x6651 : Real Time Post Processing Good To Go
#include <include/Msg_6721.h> // 0x6721 : Nortek DVL Bottom Track Data
#include <include/Msg_6722.h> // 0x6722 : Nortek DVL Water Track Data
#include <include/Msg_6723.h> // 0x6723 : Nortek Current Profile Data
#include <include/Msg_6724.h> // 0x6724 : Current Profile Velocity Data
#include <include/Msg_6725.h> // 0x6725 : Current Profile Amplitude Data
#include <include/Msg_6726.h> // 0x6726 : Current Profile Correlation Data
#include <include/Msg_6738.h> // 0x6738 : DVL Kalman Filter Data
#include <include/Msg_9900.h> // 0x9900 : Log Message
#include <include/Msg_9910.h> // 0x9910 : Processor Loading
#include <include/Msg_CE01.h> // 0xCE01 : HGuide Console Command
#include <include/Msg_FE01.h> // 0xFE01 : Read and Write Memory Message
#include <include/Msg_FE02.h> // 0xFE02 : Customer Data, Unique ID
#include <include/Msg_01.h> // 0x01 : Control Message
#include <include/Msg_02.h> // 0x02 : Navigation Message
#include <include/Msg_04.h> // 0x04 : Control Message
#include <include/Msg_05.h> // 0x05 : Navigation Message
#include <include/Msg_0C.h> // 0x0C : Control Message
#include <include/Msg_0D.h> // 0x0D : Navigation Message
#include <include/Msg_A1.h> // 0xA1 : Control Message
#include <include/Msg_A2.h> // 0xA2 : Navigation Message
#include <include/Msg_A3.h> // 0xA3 : Navigation Message without Control Data
#include <include/Msg_A9.h> // 0xA9 : Navigation Message with magnetometer flux
#include <include/Msg_AC.h> // 0xAC : Control Message with Magnetometer data
#include <include/Msg_AD.h> // 0xAD : Navigation Message with Magnetometer data
#include <include/Msg_AE.h> // 0xAE : Navigation Message with Magnetometer data and without Control Data
#include <include/Msg_CA.h> // 0xCA : Control Message

#endif // __HGuideAPI_master_h__
