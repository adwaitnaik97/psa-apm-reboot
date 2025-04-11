#ifndef __HGuideAPI_gps_mode_table_t_h__
#define __HGuideAPI_gps_mode_table_t_h__
#pragma once

// List of GPS Modes
// RTK modes are accessible automatically when supplied with RTCMv3+ corrections
enum gps_mode_table_t
{
	GPS_MODE_0_STANDALONE   = 0, //  Basic GNSS Signal
	GPS_MODE_1_SBAS         = 1, //  Satelite based augmentation system
	GPS_MODE_2_CODE_DGPS    = 2, //  Differential GPS
	GPS_MODE_3_RTK_FLOAT    = 3, //  RTK Float accuracy
	GPS_MODE_4_RTK_FIXED    = 4, //  RTK Fixed accuracy
	GPS_MODE_5_PPP_CONVERGE = 5, //  Converging PPP Solution
	GPS_MODE_6_PPP_LOCK     = 6, //  PPP Locked solution
	GPS_MODE_7_RTK_WIDELANE = 7, //  RTK Widelane accuracy
	GPS_MODE_15_INVALID     = 15 //  Invalid GNSS data
};

#endif // __HGuideAPI_gps_mode_table_t_h__
