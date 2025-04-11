# 0xF5 : Status Message - Gyro Configuration

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	float32 gyro_range  # Gyro Range
	filter_config_t gyro_config  # Gyro Filter Configuration
	uint16 gyro_sample_rate  # Gyro Sample Rate
	uint16 Checksum  # Uint16 Checksum. Complement = False
