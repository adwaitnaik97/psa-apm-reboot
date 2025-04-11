# 0xF9 : Status Message - CPU Utilization

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	float32 percent_fr  # percent frame rate
	float32 percent_ffr  # percent filter frame rate
	float32 percent_cr  # percent control rate
	float32 percent_gr  # percent guidance rate
	float32 percent_100_fr  # percent 100Hz frame rate
	float32 percent_10_fr  # percent 10Hz frame rate
	float32 percent_1_fr  # percent 1Hz frame rate
	uint16 Checksum  # Uint16 Checksum. Complement = False
