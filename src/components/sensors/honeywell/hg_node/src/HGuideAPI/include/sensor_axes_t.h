#ifndef __HGuideAPI_sensor_axes_t_h__
#define __HGuideAPI_sensor_axes_t_h__
#pragma once

//
// Sensor Axes 
//
struct sensor_axes_t
{
	bool accel_x; // 1 = ACTIVE || 0 = INACTIVE
	bool accel_y; // 1 = ACTIVE || 0 = INACTIVE
	bool accel_z; // 1 = ACTIVE || 0 = INACTIVE
	bool gyro_x; // 1 = ACTIVE || 0 = INACTIVE
	bool gyro_y; // 1 = ACTIVE || 0 = INACTIVE
	bool gyro_z; // 1 = ACTIVE || 0 = INACTIVE
	bool mag_x; // 1 = ACTIVE || 0 = INACTIVE
	bool mag_y; // 1 = ACTIVE || 0 = INACTIVE
	bool mag_z; // 1 = ACTIVE || 0 = INACTIVE

	void Default()
	{
		accel_x = 0;
		accel_y = 0;
		accel_z = 0;
		gyro_x = 0;
		gyro_y = 0;
		gyro_z = 0;
		mag_x = 0;
		mag_y = 0;
		mag_z = 0;
	}
};

#endif // __HGuideAPI_sensor_axes_t_h__
