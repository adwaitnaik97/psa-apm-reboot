# 0xF8 : Status Message - Power Information

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	float32 device_current  # Device Input Current
	float32 device_voltage  # Device Input Voltage
	uint16 Checksum  # Uint16 Checksum. Complement = False
