#ifndef __HGuideAPI_ins_gnss_summary_t_h__
#define __HGuideAPI_ins_gnss_summary_t_h__
#pragma once

#include <include/ins_mode_table_t.h>
#include <include/gps_mode_table_t.h>

// INS / GNSS Summary Word
//
// Common status describing INS and GNSS Modes along with brief information about system failures
// The data is synchronized accross all messages in a single frame.
//
struct ins_gnss_summary_t
{
	ins_mode_table_t INSMode; // Current INS Mode of operation
	bool INSStatus; // INS System Status (0 = OK | 1 = Fail)
	bool IMUStatus; // IMU System Status (0 = OK | 1 = Fail)
	bool GNSSStatus; // GNSS System Status (0 = OK | 1 = Fail)
	bool MotionDetectActive; // Motion Detection Algorithm (0 = Not Active | 1 = Active)
	bool StationaryMeasurementsOn; // ZUPT received from external source (0x1002 message)
	bool MDT1RotationRate; // Angular Rate limit (0 = not met | 1 = met)
	bool MDT2SpeedSTDV; // Velocity limit (0 = not met | 1 = met)
	bool MDT3AngularRateInstantBit; // Angular Rate limit (0 = not met | 1 = met)
	bool MDT4LinearAccelerationBit; // Linear Acceleration limit (0 = not met | 1 = met)
	bool MDT5OdometerBit; // DMI ZUPT received (0 = not received | 1 = received)
	bool MDNavigationMode; // All Motion Detect Tests are set - wating for settling time (5s = default) (0 = not set | 1 = set)
	
	// Blended Navigation Smoothing Status:
	//0 = Smooth output (Filter resets are smoothed by a defined time constant)
	//1 = Reserved
	//2 = Raw Filter output (Visible filter resets)
	//7 = Undefined
	uint8_t NavSmoothingStatus;
	gps_mode_table_t GPSMode; // Current GNSS Mode of operation

	void Default()
	{
		INSMode = INS_MODE_STANDBY;
		INSStatus = 0;
		IMUStatus = 0;
		GNSSStatus = 0;
		MotionDetectActive = 0;
		StationaryMeasurementsOn = 0;
		MDT1RotationRate = 0;
		MDT2SpeedSTDV = 0;
		MDT3AngularRateInstantBit = 0;
		MDT4LinearAccelerationBit = 0;
		MDT5OdometerBit = 0;
		MDNavigationMode = 0;
		NavSmoothingStatus = 7;
		GPSMode = GPS_MODE_0_STANDALONE;
	}
};

#endif // __HGuideAPI_ins_gnss_summary_t_h__
