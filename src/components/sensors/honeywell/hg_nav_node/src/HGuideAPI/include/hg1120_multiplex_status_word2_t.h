#ifndef __HGuideAPI_hg1120_multiplex_status_word2_t_h__
#define __HGuideAPI_hg1120_multiplex_status_word2_t_h__
#pragma once

//
// A custom data type which should be used to define a message which contains
// Multiplex status word of the HG1120 IMU 
// the multiplex variable is located in the main status word
//
// Word = 2 bytes
// MuxStatusCounter 4 and 10-15 reserved
//
struct hg1120_multiplex_status_word2_t
{
	uint16_t EmbeddedSoftwareVersion; // Embedded SW Major Version [Integer value]
	bool SensorElectronicsStatus; // 0=OK, 1=Fail
	bool SensorDataReadyStatus; // 0=OK, 1=Fail
	bool TemperatureStatus; // 0=OK, 1=Fail
	bool AccelerometerXHealth; // 0=OK, 1=Fail
	bool AccelerometerYHealth; // 0=OK, 1=Fail
	bool AccelerometerZHealth; // 0=OK, 1=Fail
	bool GyroXHealth; // 0=OK, 1=Fail
	bool GyroYHealth; // 0=OK, 1=Fail
	bool GyroZHealth; // 0=OK, 1=Fail
	bool TemperatureStatusLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 5 reset by power cycle)
	bool AccelerometerXHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool AccelerometerYHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool AccelerometerZHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroXHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroYHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroZHealthLatched; // 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool MagnetometerXHealth; // 0=OK, 1=Fail
	bool MagnetometerYHealth; // 0=OK, 1=Fail
	bool MagnetometerZHealth; // 0=OK, 1=Fail
	bool LoopCompletionTest; // 0=OK, 1=Fail
	bool RAMTest; // 0=OK, 1=Fail
	bool CoefficientTableCRCTest; // 0=OK, 1=Fail
	bool ConfigurationTableCRCTest; // 0=OK, 1=Fail
	bool NormalModeSWCRCTest; // 0=OK, 1=Fail
	bool StackOverflowTest; // 0=OK, 1=Fail
	bool WatchdogTimerTest; // 0=OK, 1=Fail
	bool ProcessorTest; // 0=OK, 1=Fail
	bool LoopCompletionTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool RAMTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool CoefficientTableCRCTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool ConfigurationTableCRCTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool NormalModeSWCRCTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool StackOverflowTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool WatchdogTimerTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool ProcessorTestLatched; // 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	float SensorTemperature; // [deg C] Accelerometer and Gyroscope sensor temperature - Not Calibrated
	float MagnetometerTemperature; // [deg C] Magnetometer temperature - Not Calibrated
	bool DIO1; // DIO 1 Configuration input value at startup echo
	bool DIO2; // DIO 2 Configuration input value at startup echo
	bool DIO3; // DIO 3 Configuration input value at startup echo
	bool DIO4; // DIO 4 Configuration input value at startup echo

	void Default()
	{
		EmbeddedSoftwareVersion = 0;
		SensorElectronicsStatus = 0;
		SensorDataReadyStatus = 0;
		TemperatureStatus = 0;
		AccelerometerXHealth = 0;
		AccelerometerYHealth = 0;
		AccelerometerZHealth = 0;
		GyroXHealth = 0;
		GyroYHealth = 0;
		GyroZHealth = 0;
		TemperatureStatusLatched = 0;
		AccelerometerXHealthLatched = 0;
		AccelerometerYHealthLatched = 0;
		AccelerometerZHealthLatched = 0;
		GyroXHealthLatched = 0;
		GyroYHealthLatched = 0;
		GyroZHealthLatched = 0;
		MagnetometerXHealth = 0;
		MagnetometerYHealth = 0;
		MagnetometerZHealth = 0;
		LoopCompletionTest = 0;
		RAMTest = 0;
		CoefficientTableCRCTest = 0;
		ConfigurationTableCRCTest = 0;
		NormalModeSWCRCTest = 0;
		StackOverflowTest = 0;
		WatchdogTimerTest = 0;
		ProcessorTest = 0;
		LoopCompletionTestLatched = 0;
		RAMTestLatched = 0;
		CoefficientTableCRCTestLatched = 0;
		ConfigurationTableCRCTestLatched = 0;
		NormalModeSWCRCTestLatched = 0;
		StackOverflowTestLatched = 0;
		WatchdogTimerTestLatched = 0;
		ProcessorTestLatched = 0;
		SensorTemperature = 0;
		MagnetometerTemperature = 0;
		DIO1 = 0;
		DIO2 = 0;
		DIO3 = 0;
		DIO4 = 0;
	}
};

#endif // __HGuideAPI_hg1120_multiplex_status_word2_t_h__
