# 0xB4 : Attitude Message

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint16 session  # Session ID
	float64 systemTov  # Time of Validity
	float32 attitude_i  # Attitude quaternion i component
	float32 attitude_j  # Attitude quaternion j component
	float32 attitude_k  # Attitude quaternion k component
	float32 attitude_s  # Attitude quaternion s component
	float32 temperature  # Temperature
	status_word_t status_word  # Validity Bit Field
	uint16 Checksum  # Uint16 Checksum. Complement = False
