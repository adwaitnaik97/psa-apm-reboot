#ifndef __HGuideAPI_hg4930_multiplex_status_word2_t_h__
#define __HGuideAPI_hg4930_multiplex_status_word2_t_h__
#pragma once

//
// Multiplexed status word of the HG4930 IMU
// the multiplex variable is located inside this type
//
// Word = 2 bytes
//
struct hg4930_multiplex_status_word2_t
{
	bool GyroHealth1; // Test validating Gyroscope asic (0=OK, 1=Fail)
	bool StartDataFlag; // Flag indicating startup (0=normal Operation, 1=startup) with 0xFFFF transmitted for synchronization
	bool ProcessTest; // 0=OK, 1=Fail
	bool MemoryTest; // 0=OK, 1=Fail
	bool ElectronicsTest; // 0=OK, 1=Fail
	bool GyroHealth2; // Test validating Gyroscope performance (0=OK, 1=Fail)
	bool AcceHealth; // 0=OK, 1=Fail
	bool StatusWord2ID; // Status Word ID (0 = 2A, 1 = 2B)
	uint8_t EmbeddedSoftwareVersion; // Embedded SW Major Version [Integer value]
	int8_t AccelXTemperature; // [deg C] Temperature of X axis Accelerometer - Not Calibrated

	void Default()
	{
		GyroHealth1 = 0;
		StartDataFlag = 0;
		ProcessTest = 0;
		MemoryTest = 0;
		ElectronicsTest = 0;
		GyroHealth2 = 0;
		AcceHealth = 0;
		StatusWord2ID = 0;
		EmbeddedSoftwareVersion = 0;
		AccelXTemperature = 0;
	}
};

#endif // __HGuideAPI_hg4930_multiplex_status_word2_t_h__
