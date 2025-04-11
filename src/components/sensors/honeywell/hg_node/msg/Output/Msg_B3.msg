# 0xB3 : Magnetometer Message

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint16 session  # Session ID
	float64 systemTov  # Time of Validity
	float32 MagneticFieldX  # Magnetic field along X axis
	float32 MagneticFieldY  # Magnetic field along Y axis
	float32 MagneticFieldZ  # Magnetic field along Z axis
	float32 temperature  # Temperature
	status_word_t status_word  # Validity Bit Field
	uint16 Checksum  # Uint16 Checksum. Complement = False
