# 0xCA : Control Message

#
#  Used by Platform Stabilization version of HG4930
#  It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
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
	uint16 Checksum  # uint16 checksum. No complements or anything, just adds all of the data as if it were a uint16.
