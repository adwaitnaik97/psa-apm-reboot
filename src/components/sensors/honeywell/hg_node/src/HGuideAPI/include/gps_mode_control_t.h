#ifndef __HGuideAPI_gps_mode_control_t_h__
#define __HGuideAPI_gps_mode_control_t_h__
#pragma once

//
// Enumeration of available GPS Modes to command the unit to
//
enum gps_mode_control_t
{
	GPS_COMMAND_0_NO_CHANGE     = 0, //  No change to the current GPS mode
	GPS_COMMAND_1_MODE_TO_INIT  = 1,
	GPS_COMMAND_2_MODE_TO_TRACK = 2
};

#endif // __HGuideAPI_gps_mode_control_t_h__
