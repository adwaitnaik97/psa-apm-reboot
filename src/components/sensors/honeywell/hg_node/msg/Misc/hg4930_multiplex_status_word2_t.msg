#
# Multiplexed status word of the HG4930 IMU
# the multiplex variable is located inside this type
#
# Word = 2 bytes
#
# custom hg4930_multiplex_status_word2_t
	bool GyroHealth1  # Test validating Gyroscope asic (0=OK, 1=Fail)
	bool StartDataFlag  # Flag indicating startup (0=normal Operation, 1=startup) with 0xFFFF transmitted for synchronization
	bool ProcessTest  # 0=OK, 1=Fail
	bool MemoryTest  # 0=OK, 1=Fail
	bool ElectronicsTest  # 0=OK, 1=Fail
	bool GyroHealth2  # Test validating Gyroscope performance (0=OK, 1=Fail)
	bool AcceHealth  # 0=OK, 1=Fail
	bool StatusWord2ID  # Status Word ID (0 = 2A, 1 = 2B)
	uint8 EmbeddedSoftwareVersion  # Embedded SW Major Version [Integer value]
	int8 AccelXTemperature  # [deg C] Temperature of X axis Accelerometer - Not Calibrated
