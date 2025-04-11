# 0xF2 : Status Message - Hardware Information

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint32 part_number  # Part Number
	uint8[4] hardware_version  # Hardware Version
	uint16 Checksum  # Uint16 Checksum. Complement = False
