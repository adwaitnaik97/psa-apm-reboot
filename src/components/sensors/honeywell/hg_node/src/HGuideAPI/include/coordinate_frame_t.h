#ifndef __HGuideAPI_coordinate_frame_t_h__
#define __HGuideAPI_coordinate_frame_t_h__
#pragma once

//
// Coordinate Frame enumeration
//
enum coordinate_frame_t
{
	INVALID_FRAME = 0,
	VEHICLE_BODY  = 1, //  Vehicle Origin
	CASE_BODY     = 2, //  Case Frame Origin
	GNSS_BODY     = 3, //  RF1 (Main) Antenna Reference Point. Attitude defined by the RF1-RF2 baseline heading.
	IMU_BODY      = 4 //  IMU frame - the IMU can be rotated in the INS case
};

#endif // __HGuideAPI_coordinate_frame_t_h__
