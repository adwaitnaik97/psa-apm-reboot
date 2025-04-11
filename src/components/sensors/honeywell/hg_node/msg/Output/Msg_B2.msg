# 0xB2 : Guidance Message

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint16 session  # Session ID
	float64 systemTov  # Time of Validity
	float64 DeltaAngleX  # Delta Angle on X axis
	float64 DeltaAngleY  # Delta Angle on Y axis
	float64 DeltaAngleZ  # Delta Angle on Z axis
	float64 DeltaVelocityX  # Delta Velocity on X axis
	float64 DeltaVelocityY  # Delta Velocity on Y axis
	float64 DeltaVelocityZ  # Delta Velocity on Z axis
	float32 temperature  # Temperature
	status_word_t status_word  # Device status information
	uint16 Checksum  # uint16 checksum
