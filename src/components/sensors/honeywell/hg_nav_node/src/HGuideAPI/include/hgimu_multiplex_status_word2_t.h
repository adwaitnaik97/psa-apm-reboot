#ifndef __HGuideAPI_hgimu_multiplex_status_word2_t_h__
#define __HGuideAPI_hgimu_multiplex_status_word2_t_h__
#pragma once

//
// Multiplexed status word for IMU where the multiplex variable is taken from the 
// the HGuide IMU Status Word #1 
// it contains various status information
// Word = 2 bytes
//
struct hgimu_multiplex_status_word2_t
{
	uint8_t EmbeddedSoftwareVersion; // Bits 0 - 7 : Embedded SW Major Version [Integer value]
	uint8_t DeviceId; // Bits 8 - 11 : Device ID
	
	// Bits 12 - 15 : Device Performance Grade
	// 0xA = A Grade
	// 0xB = B Grade
	uint8_t PerformanceGrade;
	bool Gyro_StatisticsSummary; // Bit 0 : Gyro Stat Summary (0=OK, 1=Fail)
	bool Gyro_TemperatureSummary; // Bit 1 : Gyro Temperature Summary (0=OK, 1=Fail)
	bool Accel_StatisticsSummary; // Bit 2 : Accel Stat Summary (0=OK, 1=Fail)
	bool Accel_TemperatureSummary; // Bit 3 : Accel Temperature Summary (0=OK, 1=Fail)
	bool Mag_StatisticsSummary; // Bit 4 : Mag Stat Summary (0=OK, 1=Fail)
	bool Mag_TemperatureSummary; // Bit 5 : Mag Temperature Summary (0=OK, 1=Fail)
	bool NormalModePrimaryCRC; // Bit 0 : Normal Mode Primary CRC (0=OK, 1=Fail)
	bool NormalModeSecondayrCRC; // Bit 1 : Normal Mode Secondary CRC (0=OK, 1=Fail)
	bool FactoryConfigCRC; // Bit 2 : Factory Configuration CRC (0=OK, 1=Fail)
	bool FactoryCoefficientCRC; // Bit 3 : Factory Coefficient CRC (0=OK, 1=Fail)
	bool IO_ConfigCRC; // Bit 4 : IO Configuration CRC (0=OK, 1=Fail)
	bool PrimaryImageBoot; // Bit 10 : Primary Image Boot (0=OK, 1=Fail)
	bool MemoryTestSummary; // Bit 11 : Memory Test Summary (0=OK, 1=Fail)
	bool ProcessorTestSummary; // Bit 12 : Processor Test Summary (0=OK, 1=Fail)
	bool WdtLoopCompletionSummary; // Bit 13 : WDT/Loop Completion Summary (0=OK, 1=Fail)
	bool PowerUpBitStatus; // Bit 14 : Result of Power-up Built-in Test (BIT) (0=OK, 1=Fail)
	bool ContinuousBitStatus; // Bit 15 : Result of Continuous Built-in Test (BIT) (0=OK, 1=Fail)
	float DeviceTemperature; // Bits 0 - 15 :[deg C] Device Temperature

	void Default()
	{
		EmbeddedSoftwareVersion = 0;
		DeviceId = 0;
		PerformanceGrade = 0;
		Gyro_StatisticsSummary = 0;
		Gyro_TemperatureSummary = 0;
		Accel_StatisticsSummary = 0;
		Accel_TemperatureSummary = 0;
		Mag_StatisticsSummary = 0;
		Mag_TemperatureSummary = 0;
		NormalModePrimaryCRC = 0;
		NormalModeSecondayrCRC = 0;
		FactoryConfigCRC = 0;
		FactoryCoefficientCRC = 0;
		IO_ConfigCRC = 0;
		PrimaryImageBoot = 0;
		MemoryTestSummary = 0;
		ProcessorTestSummary = 0;
		WdtLoopCompletionSummary = 0;
		PowerUpBitStatus = 0;
		ContinuousBitStatus = 0;
		DeviceTemperature = 0;
	}
};

#endif // __HGuideAPI_hgimu_multiplex_status_word2_t_h__
