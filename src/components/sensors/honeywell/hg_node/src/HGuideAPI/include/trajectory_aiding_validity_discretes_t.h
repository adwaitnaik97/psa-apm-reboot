#ifndef __HGuideAPI_trajectory_aiding_validity_discretes_t_h__
#define __HGuideAPI_trajectory_aiding_validity_discretes_t_h__
#pragma once

// Word to define which trajectory aiding sources to be used
//
// A custom data type defining the list of possible trajectory aiding source
// the trajectory aiding sources 32-bit word .
// 1 = Valid
// 0 = Invalid
//
struct trajectory_aiding_validity_discretes_t
{
	bool LATITUDE; // Bit 0 Least significant Bit
	bool LONGITUDE; // Bit 1
	bool ALTITUDE; // Bit 2
	bool BARO_ALTITUDE; // Bit 3
	bool TRUE_AIR_SPEED; // Bit 4
	bool VELOCITY_NORTH; // Bit 5
	bool VELOCITY_EAST; // Bit 6
	bool VELOCITY_DOWN; // Bit 7
	bool VELOCITY_X; // Bit 8
	bool VELOCITY_Y; // Bit 9
	bool VELOCITY_Z; // Bit 10
	bool ATTITUDE_ROLL; // Bit 11
	bool ATTITUDE_PITCH; // Bit 12
	bool ATTITUDE_HEADING; // Bit 13
	bool ALTITUDE_TYPE; // Bit 24
	bool HEADING_TYPE; // Bit 25

	void Default()
	{
		LATITUDE = 0;
		LONGITUDE = 0;
		ALTITUDE = 0;
		BARO_ALTITUDE = 0;
		TRUE_AIR_SPEED = 0;
		VELOCITY_NORTH = 0;
		VELOCITY_EAST = 0;
		VELOCITY_DOWN = 0;
		VELOCITY_X = 0;
		VELOCITY_Y = 0;
		VELOCITY_Z = 0;
		ATTITUDE_ROLL = 0;
		ATTITUDE_PITCH = 0;
		ATTITUDE_HEADING = 0;
		ALTITUDE_TYPE = 0;
		HEADING_TYPE = 0;
	}
};

#endif // __HGuideAPI_trajectory_aiding_validity_discretes_t_h__
