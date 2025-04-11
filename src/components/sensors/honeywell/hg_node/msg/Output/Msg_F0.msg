# 0xF0 : Status Message - Device Type

#
#  Message contains device type information
# 

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	uint8[4] device_type  # Device type
	uint8[4] device_config  # Device configuration
	uint16 Checksum  # Uint16 Checksum. Complement = False
