#ifndef __HGuideAPI_ins_mode_control_t_h__
#define __HGuideAPI_ins_mode_control_t_h__
#pragma once

//
// Enumeration of available INS Modes to command the unit to
//
enum ins_mode_control_t
{
	INS_MODE_0_NO_CHANGE    = 0, //  No change to the current INS mode
	INS_MODE_1_STANDBY      = 1,
	INS_MODE_2_COARSE_LEVEL = 2,
	INS_MODE_3_UNAIDED_NAV  = 3,
	INS_MODE_4_AIDED_NAV    = 4,
	INS_MODE_9_IBIT         = 9
};

#endif // __HGuideAPI_ins_mode_control_t_h__
