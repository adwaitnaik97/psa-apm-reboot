#ifndef __HGuideAPI_ins_mode_table_t_h__
#define __HGuideAPI_ins_mode_table_t_h__
#pragma once

// List of INS modes
// defines the current INS mode of operation
enum ins_mode_table_t
{
	// INS system is waiting for position, velocity and attitude data to initialize 
	// When using single antenna send the 0x1401 with attitude to initialize
	INS_MODE_STANDBY             = 1,
	INS_MODE_COARSE_LEVEL        = 2, //  INS system is computing the ROLL and PITCH attitude
	INS_MODE_UNAIDED_NAV         = 3, //  RESERVED Free Inertial navigation
	INS_MODE_AIDED_NAV           = 4, //  Aided navigation - normal operation
	INS_MODE_AHRS                = 5, //  RESERVED AHRS functionality only
	INS_MODE_PP_AIDED_FORWARD    = 6, //  RESERVED Post processed - forward solution
	INS_MODE_PP_AIDED_REVERSE    = 7, //  RESERVED Post processed - reverse solution
	INS_MODE_PP_AIDED_COMBINED   = 8, //  RESERVED Post processed - combined solution
	INS_MODE_IBIT                = 9, //  RESERVED Internal Built-in Test
	INS_MODE_SD_CARD_MAINTENANCE = 10 //  RESERVED SD card maintanence ongoing
};

#endif // __HGuideAPI_ins_mode_table_t_h__
