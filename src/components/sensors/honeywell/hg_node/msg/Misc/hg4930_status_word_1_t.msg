#
# the HG4930 IMU Status Word #1 
# note that the status info is swapped for Gyro BIT
# use the 2-bit counter to find out if you're missing any samples
#
# custom hg4930_status_word_1_t
	uint8 Counter  # 2-bit Counter
	uint8 BITmodeIndicator  # Built-in Test indicator (0 = Power-up BIT | 1 = Continuous BIT | 2-3 = Reserved)
	bool IMU_BIT_Summary  # IMU BIT Summary (0=OK, 1=Fail) latched failure is cleared after power up or reset.
	bool Gyro_BIT_Summary  # Gyroscope BIT Summary (0=OK, 1=Fail) latched failure is cleared after power up or reset.
	bool Accel_BIT_Summary  # Accelerometer BIT Summary (0=OK, 1=Fail) latched failure is cleared after power up or reset.
	bool GyroVoltage_BIT  # Gyroscope Voltage test (0=OK, 1=Fail)
	bool GyroX_BIT  # X axis Gyroscope test (0=Fail, 1=OK)
	bool GyroY_BIT  # Y axis Gyroscope test (0=Fail, 1=OK)
	bool GyroZ_BIT  # Z axis Gyroscope test (0=Fail, 1=OK)
	bool IMU_OK  # IMU test (0=OK, 1=Fail)
