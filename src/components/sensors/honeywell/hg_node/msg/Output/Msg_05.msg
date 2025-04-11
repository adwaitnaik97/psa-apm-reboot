# 0x05 : Navigation Message

#
#  It contains the Delta Angle in Rad and Delta Velocity in m/s as well as the 1/6 of Control Data frames
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
	float32 MagneticFieldX  # [mGauss] Case Frame Linear Acceleration X
	float32 MagneticFieldY  # [mGauss] Case Frame Linear Acceleration Y
	float32 MagneticFieldZ  # [mGauss] Case Frame Linear Acceleration Z
	hg1120_status_word_1_t MainStatusWord  # Main Status Word
	hg1120_multiplex_status_word2_t MultiPlexedStatusWord  # Multiplexed Status Word
	float64 DeltaAngleX  # [rad] Case Frame Delta Angle X
	float64 DeltaAngleY  # [rad] Case Frame Delta Angle Y
	float64 DeltaAngleZ  # [rad] Case Frame Delta Angle Z
	float64 DeltaVelocityX  # [m/sec] Case Frame Delta Velocity X
	float64 DeltaVelocityY  # [m/sec] Case Frame Delta Velocity Y
	float64 DeltaVelocityZ  # [m/sec] Case Frame Delta Velocity Z
	uint16 Checksum  # uint16 checksum. No complements or anything, just adds all of the data as if it were a uint16.
