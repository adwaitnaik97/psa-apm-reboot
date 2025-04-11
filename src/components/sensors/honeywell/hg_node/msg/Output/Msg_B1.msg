# 0xB1 : Control Message

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint16 session  # Session ID
	float64 systemTov  # Time of Validity
	float32 AngularRateX  # Angular Rate on the X axis
	float32 AngularRateY  # Angular Rate on the Y axis
	float32 AngularRateZ  # Angular Rate on the Z axis
	float32 LinearAccelerationX  # Specific Force along the X axis
	float32 LinearAccelerationY  # Specific Force along the Y axis
	float32 LinearAccelerationZ  # Specific Force along the Z axis
	float32 temperature  # Device Temperature
	status_word_t status_word  # Device status information
	uint16 Checksum  # Uint16 Checksum. Complement = False
