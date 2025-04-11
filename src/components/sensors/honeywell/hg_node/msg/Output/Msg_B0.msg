# 0xB0 : 1Hz Status Message

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint8 fw_major  # Firmware Major Version
	uint8 fw_minor  # Firmware Minor Version
	uint8[4] device_type  # Device Type
	uint32 part_number  # Device Part Number
	uint32 hardware_version  # Hardware Version
	uint32 device_config  # Device Performance Grade
	uint8[8] serial_number  # Serial Number
	filter_config_t gyro_filter_config  # Gyro filter config
	filter_config_t accel_filter_config  # Gyro v Filter Numerator
	float32 gyro_range  # Gyro Range
	float32 accel_range  # Accel Range
	uint32 hours_of_operation  # Hours of Operation
	uint32 boot_count  # Boot Count
	uint32 baud_rate  # Baud Rate
	uint32 sampling_frequency  # Sampling Frequency
	uint32 bytes_transmitted  # Bytes Transmitted since last 0xB0 transmission
	float32 percent_fr  # percent frame rate
	float32 percent_ffr  # percent filter frame rate
	float32 percent_cr  # percent control rate
	float32 percent_gr  # percent guidance rate
	float32 percent_100_fr  # percent 100Hz frame rate
	float32 percent_10_fr  # percent 10Hz frame rate
	float32 percent_1_fr  # percent 1Hz frame rate
	float32 device_current  # Current Consumption
	float32 device_voltage  # Input Voltage
	sensor_axes_t active_sensor_axes  # Active Sensor Axes
	sensor_axes_t saturated_sensor_axes  # Saturated Sensor Axes
	uint16 Checksum  # Checksum. Complement = False
