# Imu Built-in Test status
# Summary status information on the internal IMU
# custom imu_bit_status_t
	bool imu_failed  # IMU Failed
	bool processor_failed  # Processor Failed
	bool memory_failed  # Memory Failed
	bool other_failed  # All others Failed
	bool accelerometer_failed  # Accelerometer Failed
	bool gyro_failed  # Gyro Failed
	bool external_sync_fault  # External Sync Fault [Reserved]
	
	# IMU BIT Mode:
	# 0 = Power Up
	# 1 = Continuous
	# 2 = Initiated
	# 3 = Undefined
	# Note: IBIT response is less than 1 second (~30 msec) and may be missed by 1 Hz message rate.
	bool imu_bit_mode
