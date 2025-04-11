# 0xF6 : Status Message - Accel Configuration

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	float32 accel_range  # Accel Range
	filter_config_t accel_config  # Accel Filter Configuration
	uint16 accel_sample_rate  # Accel Sample Rate
	uint16 Checksum  # Uint16 Checksum. Complement = False
