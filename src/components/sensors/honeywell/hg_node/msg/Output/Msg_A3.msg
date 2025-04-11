# 0xA3 : Navigation Message without Control Data

#
#  It contains the Delta Angle in Rad and Delta Velocity in m/s however no Control Data
# 

# Variables declaration
	uint8 SyncByte  # Sync Byte
	uint8 MessageID  # Message ID
	float64 DeltaAngleX  # [rad] Case Frame Delta Angle X
	float64 DeltaAngleY  # [rad] Case Frame Delta Angle Y
	float64 DeltaAngleZ  # [rad] Case Frame Delta Angle Z
	float64 DeltaVelocityX  # [m/sec] Case Frame Delta Velocity X
	float64 DeltaVelocityY  # [m/sec] Case Frame Delta Velocity Y
	float64 DeltaVelocityZ  # [m/sec] Case Frame Delta Velocity Z
	hgimu_status_word_1_t StatusWord1  # Status Word 1
	hgimu_multiplex_status_word2_t MultiPlexedStatusWord2  # Multiplexed Status Word 2
	uint16 Checksum  # uint16 checksum
