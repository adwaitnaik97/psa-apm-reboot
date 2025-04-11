# 0x0C : Control Message

#
#  It contains the Angular Rate in Rad/s and Linear Acceleration in m/s/s
#  5/6 frames are in the 0x0C, the 6th frame is in the 0x0D
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
	float32 MagneticFieldX  # [mGauss] Case Frame Magnetic Flux X
	float32 MagneticFieldY  # [mGauss] Case Frame Magnetic Flux Y
	float32 MagneticFieldZ  # [mGauss] Case Frame Magnetic Flux Z
	hg1120_status_word_1_t MainStatusWord  # Main Status Word
	hg1120_multiplex_status_word2_t MultiPlexedStatusWord  # Multiplexed Status Word
	uint16 Checksum  # uint16 checksum
