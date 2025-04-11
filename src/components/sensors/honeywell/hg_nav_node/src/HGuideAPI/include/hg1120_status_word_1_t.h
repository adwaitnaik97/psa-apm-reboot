#ifndef __HGuideAPI_hg1120_status_word_1_t_h__
#define __HGuideAPI_hg1120_status_word_1_t_h__
#pragma once

//
// Main status word of the HG1120 IMU
// use this for high-level health information
//
struct hg1120_status_word_1_t
{
	uint8_t MuxStatusCounter; // 4-bit Counter
	bool IMU_OK; // IMU test (0=OK, 1=Fail)
	bool SensorBoardInit; // IMU electronics init test (0=OK, 1=Fail)
	bool AccelXValidity; // X axis Accel test (0=OK, 1=Fail)
	bool AccelYValidity; // Y axis Accel test (0=OK, 1=Fail)
	bool AccelZValidity; // Z axis Accel test (0=OK, 1=Fail)
	bool GyroXValidity; // X axis Gyro test (0=OK, 1=Fail)
	bool GyroYValidity; // Y axis Gyro test (0=OK, 1=Fail)
	bool GyroZValidity; // Z axis Gyro test (0=OK, 1=Fail)
	bool MagnetometerValidity; // Magnetometer test (0=OK, 1=Fail)
	bool PowerUp_BIT; // Power-up Built-in test (0=OK, 1=Fail) latched failure is cleared after power up or reset
	bool Continuous_BIT; // Continuous Built-in test (0=OK, 1=Fail) latched failure is cleared after power up or reset
	bool PowerUp_test; // Power-up test indication (0=Normal Operation, 1=Power-up test running)

	void Default()
	{
		MuxStatusCounter = 0;
		IMU_OK = 0;
		SensorBoardInit = 0;
		AccelXValidity = 0;
		AccelYValidity = 0;
		AccelZValidity = 0;
		GyroXValidity = 0;
		GyroYValidity = 0;
		GyroZValidity = 0;
		MagnetometerValidity = 0;
		PowerUp_BIT = 0;
		Continuous_BIT = 0;
		PowerUp_test = 0;
	}
};

#endif // __HGuideAPI_hg1120_status_word_1_t_h__
