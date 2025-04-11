#
# A custom data type which should be used to define a message which contains
# Multiplex status word of the HG1120 IMU 
# the multiplex variable is located in the main status word
#
# Word = 2 bytes
# MuxStatusCounter 4 and 10-15 reserved
#
# custom hg1120_multiplex_status_word2_t
	uint16 EmbeddedSoftwareVersion  # Embedded SW Major Version [Integer value]
	bool SensorElectronicsStatus  # 0=OK, 1=Fail
	bool SensorDataReadyStatus  # 0=OK, 1=Fail
	bool TemperatureStatus  # 0=OK, 1=Fail
	bool AccelerometerXHealth  # 0=OK, 1=Fail
	bool AccelerometerYHealth  # 0=OK, 1=Fail
	bool AccelerometerZHealth  # 0=OK, 1=Fail
	bool GyroXHealth  # 0=OK, 1=Fail
	bool GyroYHealth  # 0=OK, 1=Fail
	bool GyroZHealth  # 0=OK, 1=Fail
	bool TemperatureStatusLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 5 reset by power cycle)
	bool AccelerometerXHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool AccelerometerYHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool AccelerometerZHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroXHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroYHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool GyroZHealthLatched  # 0=OK, 1=Fail (latched if the individual BIT test counter reaches 15 reset by power cycle)
	bool MagnetometerXHealth  # 0=OK, 1=Fail
	bool MagnetometerYHealth  # 0=OK, 1=Fail
	bool MagnetometerZHealth  # 0=OK, 1=Fail
	bool LoopCompletionTest  # 0=OK, 1=Fail
	bool RAMTest  # 0=OK, 1=Fail
	bool CoefficientTableCRCTest  # 0=OK, 1=Fail
	bool ConfigurationTableCRCTest  # 0=OK, 1=Fail
	bool NormalModeSWCRCTest  # 0=OK, 1=Fail
	bool StackOverflowTest  # 0=OK, 1=Fail
	bool WatchdogTimerTest  # 0=OK, 1=Fail
	bool ProcessorTest  # 0=OK, 1=Fail
	bool LoopCompletionTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool RAMTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool CoefficientTableCRCTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool ConfigurationTableCRCTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool NormalModeSWCRCTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool StackOverflowTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool WatchdogTimerTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	bool ProcessorTestLatched  # 0=OK, 1=Fail (Latched Until Power is Cycled or Unit is Reset)
	float32 SensorTemperature  # [deg C] Accelerometer and Gyroscope sensor temperature - Not Calibrated
	float32 MagnetometerTemperature  # [deg C] Magnetometer temperature - Not Calibrated
	bool DIO1  # DIO 1 Configuration input value at startup echo
	bool DIO2  # DIO 2 Configuration input value at startup echo
	bool DIO3  # DIO 3 Configuration input value at startup echo
	bool DIO4  # DIO 4 Configuration input value at startup echo
