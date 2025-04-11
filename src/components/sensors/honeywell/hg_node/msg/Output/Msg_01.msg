# 0x01 : Control Message

#
#  It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
#  5/6 frames are in the 0x01, the 6th frame is in the 0x02
# 

# Variables declaration
	uint8 SyncByte  # Sync Byte
	uint8 MessageID  # Message ID
	float32 AngularRateX  # [rad/sec] Case Frame Angular Rate X
	float32 AngularRateY  # [rad/sec] Case Frame Angular Rate Y
	float32 AngularRateZ  # [rad/sec] Case Frame Angular Rate Z
	float32 LinearAccelerationX  # [m/sec/sec] Case Frame Linear Acceleration X
	float32 LinearAccelerationY  # [m/sec/sec] Case Frame Linear Acceleration Y
	float32 LinearAccelerationZ  # [m/sec/sec] Case Frame Linear Acceleration Z
	hg4930_status_word_1_t StatusWord1  # Status Word 1
	hg4930_multiplex_status_word2_t MultiPlexedStatusWord2  # Multiplexed Status Word 2
	uint16 Checksum  # uint16 checksum
