#ifndef __HGuideAPI_status_word_t_h__
#define __HGuideAPI_status_word_t_h__
#pragma once

//
// Sensor Validity Status Bit Fields
// Use the sensor_axes_t type to mask out the unused axes before reading the status 
//
struct status_word_t
{
	bool gyro_x_fail; // 1 = Gyro x Fail
	bool gyro_y_fail; // 1 = Gyro y Fail
	bool gyro_z_fail; // 1 = Gyro z Fail
	bool accel_x_fail; // 1 = Accel x Fail
	bool accel_y_fail; // 1 = Accel y Fail
	bool accel_z_fail; // 1 = Accel z Fail
	bool mag_x_fail; // 1 = Mag x Fail
	bool mag_y_fail; // 1 = Mag y Fail
	bool mag_z_fail; // 1 = Mag z Fail
	uint8_t counter; // 4 Bit Counter

	void Default()
	{
		gyro_x_fail = 0;
		gyro_y_fail = 0;
		gyro_z_fail = 0;
		accel_x_fail = 0;
		accel_y_fail = 0;
		accel_z_fail = 0;
		mag_x_fail = 0;
		mag_y_fail = 0;
		mag_z_fail = 0;
		counter = 0;
	}
};

#endif // __HGuideAPI_status_word_t_h__
